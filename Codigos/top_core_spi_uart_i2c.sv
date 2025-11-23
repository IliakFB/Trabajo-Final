`timescale 1ns/10ps
`include "../../TEC_RISCV/TOP/topcore_tecriscv.sv"
`include "Status.sv"
`include "IS25WP032D.v"
`include "coverage_classes.sv"
`include "instruction_class.sv"
//`include "core_coverage_collection.sv"
`include "Reference_Model.sv"
`include "Selfcheck.sv"

`define DEBUG

module topcore_tb;

logic clk;
logic reset;
wire MISO;
logic RX_UART;
wire MOSI;
wire SCLK;
wire SCS;
wire TX_UART;
wire  [7:0] gpio;
wire [7:0] full_range_level_shifter;
wire [31:0] IS_Val;
wire [31:0] IS_Config;
wire [3:0] IS_Trigger;
logic maip;
wire [7:0] Reg_GPIO_en;
logic [7:0] Reg_GPIO_int;
wire [7:0] Reg_GPIO_out;
logic [31:0] selfcheck_pc;


//////////////////////////////////////////////////////////////////////
//Instanciamiento
//////////////////////////////////////////////////////////////////////
topcore_tecriscv uut(
	.clk(clk),
	.reset(reset),
	.MISO(MISO),
	.RX_UART(RX_UART),
	.maip(maip),
	.MOSI(MOSI),
	.SCLK(SCLK),
	.SCS(SCS),
	.TX_UART(TX_UART),
	.full_range_level_shifter(full_range_level_shifter),
	.IS_Val(IS_Val),
	.IS_Config(IS_Config),
	.IS_Trigger(IS_Trigger),
	.Reg_GPIO_en(Reg_GPIO_en),
	.Reg_GPIO_int(Reg_GPIO_int),
	.Reg_GPIO_out(Reg_GPIO_out)
	);


	IS25WP032D mem(
		.SCLK(SCLK),
		.CS(SCS),
		.SI(MOSI),
		.SO(MISO),
		.WP(1'b1),
		.SIO3(1'b1));

//////////////////////////////////////////////////////////////////////
//Variables internas y parametros
//////////////////////////////////////////////////////////////////////

	int executed_inst;
	int rx_cycle_count;
    int tx_cycle_count;
	int data_count;
    int rx_byte_count;
    int tx_byte_count;
    int sent_data_count;
    int out;
    int loading_cycles;
    int exe_cycles;
    int finish_count;
    int change_counter;
    real start_loading_time;
    real stop_loading_time;
    real start_program_time;
    real stop_program_time;
    real GPIO_timer;
	logic [7:0] uart_rx_data;
	logic even_parity;
    logic [63:0] complete_uart_rx_data;
    logic [63:0] uart_tx_data_queue [$];
    logic [63:0] data;
    logic [7:0]  tx_byte_queue [$];
    logic [7:0]  rx_byte_queue [$];
    logic loaded_data_flag;
    logic uart_busy;
    logic finish;
    logic cc_count;
    logic [31:0] past_inst;
    logic program_start;
    logic loading_start;
    logic sim_timer;
	string line;
	string exe_inst_queue [$];
	string prediction     [$];
    string inst_queue     [$];
    string csr_predict    [$];
    string uart_predict   [$];
    string spi_predict    [$];
    string uart_data      [$];
    string test_name;
    //Variables para transmision I2C
    logic ackbit;
    logic contador;
    logic [6:0] addr;
    logic [7:0] dato;
    logic Bandera;
    logic i2cbytes;
    localparam int FI2C=130; //130 para 100 K, 32 para 400 k,  13 para un 1M, 4 para 3.4 Mhz aprox 
    localparam int SDA=2;
    localparam int SCL=3;



    //////////////////////////////////////////////////////////////////////
    //Coverage Class assignment
    //////////////////////////////////////////////////////////////////////

    core_coverage_class core_cov = new(clk);

    //////////////////////////////////////////////////////////////////////
    //Instruction Class for CPI measurement
    //////////////////////////////////////////////////////////////////////

    instruction_class inst_set [47];

//////////////////////////////////////////////////////////////////////
//Task y testbench
//////////////////////////////////////////////////////////////////////
initial begin
    Bandera=1;
    out = $fopen("results.txt","w+");
    $fclose(out);
    out = $fopen("temp.txt","w+");
    $fclose(out);
    if($value$plusargs("TESTNAME=%s",test_name)) begin
        //$dumpfile({"../../../sim_files/core_sim_files/pre_synth_",test_name,"_test.vcd"});
        //$dumpvars(0,topcore_tb);
        //$fsdbDumpfile({"../../../sim_files/core_sim_files/pre_synth_",test_name,"_test.fsdb"});
        //$fsdbDumpvars(0,topcore_tb);
        out = $fopen({"Metrics_Reports/Metrics_Report_",test_name,".txt"},"w+");
        $fwrite(out,"//////////////////////Metrics Report//////////////////////////\n");
        $fwrite(out,"\n");
        $fwrite(out,"---General Information---");
        $fclose(out);
    end
    else begin
        //$dumpfile("../../../sim_files/core_sim_files/topcore_tb.vcd");
	//$fsdbDumpfile("../../../sim_files/core_sim_files/topcore_tb.fsdb");
        //$dumpvars(0,topcore_tb);
	//$fsdbDumpvars(0,topcore_tb);
    end
	clk <= 0;
	reset <= 1;
	RX_UART <= 1;
	maip <= 0; //ENTRADA NUEVA
	prediction.delete();
  	exe_inst_queue.delete();
    inst_queue.delete();
    uart_tx_data_queue.delete();
    tx_byte_queue.delete();
    csr_predict.delete();
    uart_predict.delete();
  	uart_rx_data = 0;
  	rx_cycle_count = 0;
    tx_cycle_count = 0;
  	data_count = 0;
    rx_byte_count = 0;
    tx_byte_count = 0;
    complete_uart_rx_data = 0;
    sent_data_count = 0;
    loaded_data_flag = 0;
    uart_busy = 0;
    finish = 0;
    cc_count = 0;
    past_inst = 0;
    loading_cycles = 0;
    exe_cycles = 0;
    program_start = 0;
    loading_start = 0;
    finish_count = 0;
    change_counter = 0;
    sim_timer = 1;
    GPIO_timer = 0;
    Reg_GPIO_int[0] = 0;
    Reg_GPIO_int[2] = 1;
    Reg_GPIO_int[3] = 1;
    //reference_model(inst_queue,prediction,executed_inst,exe_inst_queue,csr_predict,uart_predict,spi_predict);
    if((uart_predict.size() != 0) || (spi_predict.size() != 0)) begin
        sim_timer = 0;
    end
    //$finish;

    //////////////////////////////////////////////////////////////////////
    //CPI Report Names
    //////////////////////////////////////////////////////////////////////

    inst_set[0] = new("LUI");
    inst_set[1] = new("AUIPC");
    inst_set[2] = new("JAL");
    inst_set[3] = new("JALR");
    inst_set[4] = new("BEQ");
    inst_set[5] = new("BNE");
    inst_set[6] = new("BLT");
    inst_set[7] = new("BGE");
    inst_set[8] = new("BLTU");
    inst_set[9] = new("BGEU");
    inst_set[10] = new("LB");
    inst_set[11] = new("LH");
    inst_set[12] = new("LW");
    inst_set[13] = new("LBU");
    inst_set[14] = new("LHU");
    inst_set[15] = new("SB");
    inst_set[16] = new("SH");
    inst_set[17] = new("SW");
    inst_set[18] = new("ADDI");
    inst_set[19] = new("SLTI");
    inst_set[20] = new("SLTIU");
    inst_set[21] = new("XORI");
    inst_set[22] = new("ORI");
    inst_set[23] = new("ANDI");
    inst_set[24] = new("SLLI");
    inst_set[25] = new("SRLI");
    inst_set[26] = new("SRAI");
    inst_set[27] = new("ADD");
    inst_set[28] = new("SUB");
    inst_set[29] = new("SLL");
    inst_set[30] = new("SLT");
    inst_set[31] = new("SLTU");
    inst_set[32] = new("XOR");
    inst_set[33] = new("SRL");
    inst_set[34] = new("SRA");
    inst_set[35] = new("OR");
    inst_set[36] = new("AND");
    inst_set[37] = new("FENCE");
    inst_set[38] = new("FENCE.I");
    inst_set[39] = new("ECALL");
    inst_set[40] = new("EBREAK");
    inst_set[41] = new("CSRRW");
    inst_set[42] = new("CSRRS");
    inst_set[43] = new("CSRRC");
    inst_set[44] = new("CSRRWI");
    inst_set[45] = new("CSRRSI");
    inst_set[46] = new("CSRRCI");

end

always begin
    #25 clk=~clk;
    core_cov.clk = clk;
end

always begin
    I2CTransmission();
    //Pulse();
end


always @(posedge clk)begin
    core_cov.ctrl_fsm_state = topcore_tb.uut.TOP.central_control.cur_state[7:0];
    cpi_measurement(inst_set,inst_set);
    fork
        prueba();
    join
    //$display("Core Coverage %d",core_cov.clk);
    //core_coverage_collection(core_cov);
    uart_comparison(uart_data,uart_predict,finish);
    rx_uart_monitor(TX_UART,19200);
    if(complete_uart_rx_data == 64'h8898968088888888) begin
        if (loaded_data_flag == 0) begin
            out = $fopen("uart_data.txt","r");
            while (!$feof(out)) begin
                $fgets(line,out);
                data = line.atohex();
                uart_tx_data_queue.push_back(data);
            end
            //data = uart_tx_data_queue.pop_back();
            loaded_data_flag = 1;
        end
        if (loaded_data_flag == 1) begin
            if((uart_busy == 0) && (tx_byte_count == 0)) begin
                if(uart_tx_data_queue.size() > 0) begin
                    tx_byte_queue.delete();
                    data = uart_tx_data_queue.pop_front();
                    $display("The packet to be sent through UART is: %h", data);
                    $display("-----------------------------------");
                    tx_byte_queue.push_back(data[7:0]);
                    tx_byte_queue.push_back(data[15:8]);
                    tx_byte_queue.push_back(data[23:16]);
                    tx_byte_queue.push_back(data[31:24]);
                    tx_byte_queue.push_back(data[39:32]);
                    tx_byte_queue.push_back(data[47:40]);
                    tx_byte_queue.push_back(data[55:48]);
                    tx_byte_queue.push_back(data[63:56]);
                end
            end
            tx_uart_driver(19200,tx_byte_queue[tx_byte_count],RX_UART,uart_busy);
            if((uart_busy == 0) && (uart_tx_data_queue.size() == 0) && (tx_byte_count == 0)) begin
                $display("-----------------------------------");
                $display("Transmission has ended. Finishing test...");
                $display("-----------------------------------");
                complete_uart_rx_data = 0;
                finish = 1;
            end
        end
    end
end

task prueba();
  if ($time<4000000)begin
    reset <=1;
    maip <= 0; //ENTRADA NUEVA
  end
  else begin
    reset <=0;
    out = $fopen("temp.txt","r+");
    while(!$feof(out)) begin
        $fgets(line,out);
    end
    $fclose(out);
    if(loading_start == 0) begin
        start_loading_time = $realtime();
        loading_start = 1;
    end
    if(program_start == 0) begin
        loading_cycles++;
    end
    else begin
        exe_cycles++;
    end
    if((topcore_tb.uut.TOP.central_control.cur_state[7:0] != 0) && (program_start == 0)) begin
        stop_loading_time = $realtime() - 100;
        start_program_time = $realtime() - 100;
        Bandera=1;
        program_start = 1;
        
    end
    /*if(last_inst == ) begin
        if(topcore_tb.uut.TOP.central_control.cur_state[7:0] == 4) begin
            finish_count++;
            $display("FINISH");
            $display(finish_count);
            $display($realtime());
        end
    end*/
    if((($time > 700000000) || (line == "FINISH")) && (sim_timer == 1)/*(finish == 1)*//*(finish_count >= 1)*/) begin

        //This line is for the coremark test only, comment if it is not a coremark test
    //if(topcore_tb.uut.TOP.Memoria_8K.A == 17131) begin
        ///////////////////////////////////////////////////

        stop_program_time = $realtime() - 100;
        //selfcheck(inst_queue,prediction,executed_inst,exe_inst_queue,csr_predict);
        if($value$plusargs("TESTNAME=%s",test_name)) begin
            print_metrics_report();
        end
        else begin
            $display("WARNING: Plusargs +TESTNAME=<name> is missing. Metrics reports won't be generated.");
        end
    	//$finish;
    end
    else if(((finish == 1) && (sim_timer == 0))) begin //uart test running and needs more time
        stop_program_time = $realtime() - 100;
        //selfcheck(inst_queue,prediction,executed_inst,exe_inst_queue,csr_predict);
        if($value$plusargs("TESTNAME=%s",test_name)) begin
            print_metrics_report();
        end
        else begin
            $display("WARNING: Plusargs +TESTNAME=<name> is missing. Metrics reports won't be generated.");
        end
        $finish;
    end
    if (($realtime() - GPIO_timer) >= 400000) begin
	GPIO_timer = $realtime();
	Reg_GPIO_int[0] =! Reg_GPIO_int[0];
    end
  end
endtask : prueba



task Pulse();
     begin
        // Espera inicial opcional 
        $display("Iniciando pulso simple en GPIOs");

        // Poner SCL y SDA en 1
        Reg_GPIO_int[SCL] = 1'b1;
        Reg_GPIO_int[SDA] = 1'b1;
        $display("Reg_GPIO_int (HIGH) = %08b", Reg_GPIO_int);

        // Mantenerlos en 1 cierto tiempo (por ejemplo FI2C ciclos de clk)
        repeat (FI2C) @(posedge clk);

        // Poner SCL y SDA en 0
        Reg_GPIO_int[SCL] = 1'b0;
        Reg_GPIO_int[SDA] = 1'b0;
        $display("Reg_GPIO_int (LOW)  = %08b", Reg_GPIO_int);

        // Espera final opcional
        repeat (FI2C) @(posedge clk);

        $display("Termina SimpleGpioPulse");
    end
endtask : Pulse











task I2CTransmission();
    if(Bandera==1)begin
        #6200000
        $display("Iniciando Transmision i2c",);
        addr=7'b0100010; //3C + 0 read/write 
        dato=8'b10101110; //AE
        repeat (FI2C) @(posedge clk); 
        Reg_GPIO_int[SCL] = 1'b1; //Estado esperando
        Reg_GPIO_int[SDA] = 1'b1;
        $display("Reg_GPIO_int = %08b", Reg_GPIO_int);
        repeat (FI2C) @(posedge clk);
        Reg_GPIO_int[SDA]=1'b0; //Inicia la transmision en este punto
        repeat(FI2C) @(posedge clk); 
        Reg_GPIO_int[SCL]=1'b0; //Flanco Negativo SCL, Ajusto Datos
        for(int i=6; i>= 0; i--)begin
            $display("Enviando bit de address: ", addr[i]);
            Reg_GPIO_int[SDA]=addr[i];
            repeat(FI2C)@(posedge clk);  
            Reg_GPIO_int[SCL]=1'b1 ; //Subo SCL para que el slave muestree
            repeat(FI2C)@(posedge clk);
            Reg_GPIO_int[SCL]=1'b0;
            end  //Flanco Negativo SCL, Ajusto Datos
        $display("Enviando bit de write: 0");
        Reg_GPIO_int[SDA]=0'b0;
        repeat(FI2C)@(posedge clk);
        Reg_GPIO_int[SCL]=1'b1 ; //Subo SCL para que el slave muestree
        $display("Termino envio de direccion");
        repeat(FI2C)@(posedge clk);
        Reg_GPIO_int[SCL]=0'b0; //Bajar SCL para cambio de SDA desde el esclavo
        repeat(FI2C)@(posedge clk);
        Reg_GPIO_int[SCL]=1'b1; //Muestreo de ACK
        repeat(FI2C)@(posedge clk); 
        $display("Empieza Envio de datos");
        Reg_GPIO_int[SCL]=0'b0;
        for(int j=7; j>=0; j--)begin
            $display("Enviando bit de datos: ", dato[j]);
            Reg_GPIO_int[SDA]=dato[j]; //Seteo el bit de dato
            repeat(FI2C)@(posedge clk);  
            Reg_GPIO_int[SCL]=1'b1 ; //Subo SCL para que el slave muestree
            repeat(FI2C)@(posedge clk);
            Reg_GPIO_int[SCL]=0'b0;
         end
        $display("Termina envio de datos");
        repeat(FI2C)@(posedge clk);    
        Reg_GPIO_int[SCL]=1'b1;
        repeat(FI2C)@(posedge clk);    
        Reg_GPIO_int[SCL]=0'b0; //Bajo SCL para que slave mande bit de ack
        repeat(FI2C)@(posedge clk);
        Reg_GPIO_int[SCL]=1'b1; //Muestreo de ACK
        //------------------Aqui inicia STOP
        repeat(FI2C)@(posedge clk);
        Reg_GPIO_int[SCL]=0'b0;
        repeat(FI2C)@(posedge clk);
        //Aqui se pone SDA en alto mientras SCL esta en alto, para mandar el stop
        Reg_GPIO_int[SCL]=1'b1;
        repeat(FI2C)@(posedge clk);
        Reg_GPIO_int[SCL]=0'b0;
        repeat(FI2C)@(posedge clk);
        Reg_GPIO_int[SCL]=1'b1;
        repeat(FI2C/2)@(posedge clk);
        Reg_GPIO_int[SDA]=1'b1;
        $display("Termina Rutina I2C");
        //------------------Termina rutina
    end
         
endtask : I2CTransmission















task rx_uart_monitor(input logic rx_bit, input int baud_rate); //Even parity bit is used

    int total_cycle;

    string str;

    total_cycle = 20000000/baud_rate; //20MHZ / baud_rate

    if ((rx_bit == 0) && (data_count == 0)) begin // Start Bit
    	if(total_cycle == rx_cycle_count) begin
            sim_timer = 0;
    		$display("Start bit found!");
    		rx_cycle_count = 0;
    		data_count++;
    	end
    	else begin
    		rx_cycle_count++;
    	end
    end
   	else if ((data_count > 0) && (data_count < 9)) begin // 8bit Data
   		if(total_cycle == rx_cycle_count) begin
   			$display("Data bit found: %h",(data_count - 1));
   			uart_rx_data[data_count - 1] = rx_bit;
    		rx_cycle_count = 0;
    		data_count++;
    	end
    	else begin
    		rx_cycle_count++;
    	end
    end
    else if(data_count == 9) begin // Even Parity bit
    	if(total_cycle == rx_cycle_count) begin
    		$display("Parity bit found: %b",rx_bit);
    		even_parity = rx_bit;
    		uart_rx_data[data_count - 1] = rx_bit;
    		rx_cycle_count = 0;
    		data_count++;
    	end
    	else begin
    		rx_cycle_count++;
    	end
    end
    else if((rx_bit == 1) && (data_count == 10)) begin
    	if(total_cycle == rx_cycle_count) begin
    		$display("Stop bit has been found!!");
    		$display("-----------------------------------");
			$display("UART Transaction Succesful...");
			$display("-----------------------------------");
			$display("Data received (hex): %h",uart_rx_data);
            $display("Data received (bin): %b",uart_rx_data);
			$display("Parity bit received: %b",even_parity);
			if(even_parity == (^uart_rx_data)) begin
				$display("The parity check bit is correct");
                $display("-----------------------------------");
                rx_byte_queue.push_back(uart_rx_data);
                if(rx_byte_count < 7) begin
                    rx_byte_count++;
                end
                else begin
                    complete_uart_rx_data = {rx_byte_queue[7],rx_byte_queue[6],rx_byte_queue[5],rx_byte_queue[4],rx_byte_queue[3],rx_byte_queue[2],rx_byte_queue[1],rx_byte_queue[0]};
                    $display("The packet recieved is: %h",complete_uart_rx_data);
                    $display("-----------------------------------");
                    str.hextoa(complete_uart_rx_data);
                    uart_data.push_back(str);
                    rx_byte_count = 0;

                end
			end
			else begin
				$display("Error in the parity bit");
			end;
    		rx_cycle_count = 0;
    		data_count = 0;
    	end
    	else begin
    		rx_cycle_count++;
    	end
    end

endtask : rx_uart_monitor

task tx_uart_driver(input int baud_rate, input logic [7:0] uart_tx_data, output logic tx_bit, output logic busy);

    int total_cycle;
    logic [31:0] data;

    total_cycle = 20000000/baud_rate; //20MHZ / baud_rate

    if(sent_data_count == 0) begin
        if (tx_cycle_count < total_cycle) begin //Start bit
            tx_bit = 0;
            tx_cycle_count++;
        end
        else begin
            $display("-----------------------------------");
            $display("Sending data through UART...");
            $display("Byte to be sent: %h",uart_tx_data);
            $display("-----------------------------------");
            sent_data_count++;
            tx_cycle_count = 0;
        end
        busy = 1;
    end
    else if((sent_data_count > 0) && (sent_data_count < 9)) begin
        if (tx_cycle_count < total_cycle) begin //Data bits
            tx_bit = uart_tx_data[(sent_data_count - 1)];
            tx_cycle_count++;
        end
        else begin
            sent_data_count++;
            tx_cycle_count = 0;
        end
        busy = 1;
    end
    else if(sent_data_count == 9) begin // Even Parity bit
        if(tx_cycle_count < total_cycle) begin
            tx_bit = ^uart_tx_data;
            tx_cycle_count++;
        end
        else begin
            sent_data_count++;
            tx_cycle_count = 0;
        end
        busy = 1;
    end
    else if(sent_data_count > 9) begin // Stop bit
        if(tx_cycle_count < total_cycle) begin
            tx_bit = 1;
            tx_cycle_count++;
            busy = 1;
        end
        else begin
            sent_data_count = 0;
            tx_cycle_count = 0;
            $display("-----------------------------------");
            $display("UART Transaction Succesful...");
            $display("-----------------------------------");
            busy = 0;
            if(tx_byte_count < 7) begin
                tx_byte_count++;
            end
            else begin
                tx_byte_count = 0;
            end
        end
    end

endtask : tx_uart_driver

task uart_comparison(input string real_uart [$], input string uart_prediction [$], output finish_flg);

    string str1;
    string str2;
    string filler;

    bit fail;

    int file_s;

    fail = 0;
    if (real_uart.size() != 0) begin    
        if(real_uart.size() == uart_prediction.size()) begin
            finish_flg = 1;
            if($value$plusargs("TESTNAME=%s",test_name)) begin
                file_s = $fopen({"UART_Results/",test_name,"_UART_Results.txt"},"w+");
                $fwrite(file_s,"//////////////////////UART Report//////////////////////////\n");
                $fwrite(file_s,"\n");
                $fwrite(file_s,"---Data received through the UART port---\n");
                $fwrite(file_s,"Prediction | RTL Result\n");
                $fclose(file_s);
            end
            while((real_uart.size() != 0) && (uart_prediction.size() != 0)) begin
                //$display("size",str_queue1.size());
                //$display("size",str_queue2.size());
                str1 = real_uart.pop_front();
                str2 = uart_prediction.pop_front();
                if(str1 != str2) begin
                  $display("There has been a mismatch with the UART data received...");
                  $display({"Expected UART data: ",str1});
                  $display({"REceived UART data: ",str2});
                  fail = 1;
                end
                if($value$plusargs("TESTNAME=%s",test_name)) begin
                    file_s = $fopen({"UART_Results/",test_name,"_UART_Results.txt"},"r+");
                    while(!$feof(file_s)) begin
                        $fgets(filler,file_s);
                    end
                    $fwrite(file_s,{str1,"   | ",str2,"\n"});
                    $fclose(file_s);
                end
            end
            if(real_uart.size() != uart_prediction.size()) begin
                $display("There has been a mismatch with the UART data received...");
                $display("There is some missing data. Prediction and real UART data queues are no the same!");
                fail = 1;
            end
        end
        else begin
            finish_flg = 0;
        end
    end
    if($time > 400000000) begin
        fail = 1;
        $display("UART Timeout Error");
    end
    if (fail == 1) begin
        $display("---------------------------------------",);
        $display("----------------TEST FAIL--------------");
        $display("---------------------------------------",);
        if($test$plusargs("SUMMARY")) begin
          file_s = $fopen("Summary.txt","r+");
          while(!$feof(file_s)) begin
            $fgets(filler,file_s);
          end
          filler = "";
          while (filler.len() < (30 - test_name.len())) begin //arbitrary number, it is just for aesthetics
              filler = {filler," "};
          end
          $fwrite(file_s,{" ",test_name,filler,"| FAIL\n"});
          $fclose(file_s);
        end
        $finish;
      end // if (fail == 1)

endtask : uart_comparison

task cpi_measurement(input instruction_class input_inst_set[47], output instruction_class output_inst_set[47]);

    int cc_counter;
    int position;

    logic [31:0] current_inst;

    instruction_class local_inst = new("");

    position = 0;
    current_inst = topcore_tb.uut.TOP.Memory_controller.d_read[31:0];
    //$display("Inst %d",position);
    //$display("Position %d",position);
    if(topcore_tb.uut.TOP.central_control.cur_state[7:0] == 4) begin
        change_counter++;
    end
    if (((past_inst != current_inst) && (current_inst[31:0] != 0)) || (change_counter == 3)) begin
        if (cc_count == 1) begin
            cc_counter++;
            find_inst_in_set(past_inst,position);
            //$display("Position %d",position);
            local_inst = input_inst_set[position];
            local_inst.increment_cpi(cc_counter);
            //$display("cpi %d",local_inst.cpi);
            //$display("repetitions %d",local_inst.repetitions);
            output_inst_set[position] = local_inst;
        end
        cc_counter = 0;
        cc_count = 1;
        change_counter = 0;
        past_inst = current_inst;
    end
    else if ((past_inst == current_inst) && (cc_count == 1)) begin
        cc_counter++;
        output_inst_set = input_inst_set;
    end
    else begin
        output_inst_set = input_inst_set;
    end

endtask : cpi_measurement

task print_metrics_report();

    string filler;
    string avg_cpi;
    string program_size;
    string inst_count;
    string exe_inst;
    string loading_time;
    //string str1;
    //string str2;
    string exe_time;
    string loading_cycles_str;
    string exe_cycles_str;

    program_size.itoa(((inst_queue.size()/4) - 1) * 4);
    inst_count.itoa((inst_queue.size()/4) - 1);
    exe_inst.itoa(executed_inst);
    loading_time = $sformatf("%f",(stop_loading_time - start_loading_time));
    exe_time = $sformatf("%f",(stop_program_time - start_program_time));
    //str1 = $sformatf("%f",(start_loading_time));
    //str2 = $sformatf("%f",(stop_loading_time));
    loading_cycles_str.itoa(loading_cycles);
    exe_cycles_str.itoa(exe_cycles);
    out = $fopen({"Metrics_Reports/Metrics_Report_",test_name,".txt"},"r+");
    while(!$feof(out)) begin
        $fgets(filler,out);
    end
    $fwrite(out,"\n");
    $fwrite(out,{"Program Size: ",program_size,"B"});
    $fwrite(out,"\n");
    $fwrite(out,"Program Size (Instruction Quantity): ",inst_count);
    $fwrite(out,"\n");
    $fwrite(out,"Total Executed Instructions: ",exe_inst);
    $fwrite(out,"\n");
    $fwrite(out,{"Loading Program Time: ",loading_time,"ns"});
    $fwrite(out,"\n");
    /*$fwrite(out,{"Execution Program Time: ",str1,"ns"});
    $fwrite(out,"\n");
    $fwrite(out,{"Execution Program Time: ",str2,"ns"});
    $fwrite(out,"\n");*/
    $fwrite(out,"Loading Program Cycle Quantity: ",loading_cycles_str);
    $fwrite(out,"\n");
    $fwrite(out,{"Execution Program Time: ",exe_time,"ns"});
    $fwrite(out,"\n");
    $fwrite(out,"Execution Program Cycle Quantity: ",exe_cycles_str);
    $fwrite(out,"\n");
    $fwrite(out,"\n");
    $fwrite(out,"--- CPI Measurement Results---");
    $fwrite(out,"\n");
    $fwrite(out,"Instruction | Average CPI");
    foreach(inst_set[i]) begin
        inst_set[i].calculate_average_cpi();
        avg_cpi = $sformatf("%f",inst_set[i].average_cpi);
        filler = "";
        while (filler.len() < (8 - inst_set[i].name.len())) begin //arbitrary number, it is just for aesthetics
            filler = {filler," "};
        end
        $fwrite(out,"\n");
        $fwrite(out,{"   ",inst_set[i].name,filler," | "," ",avg_cpi});
    end

    $fclose(out);

endtask : print_metrics_report

task find_inst_in_set(input logic [31:0] local_inst, output int position);

    string inst_name;

    logic [6:0]  opcode;
    logic [2:0]  funct3;

    opcode = local_inst[6:0];
    funct3 = local_inst[14:12];

    case (opcode)
         7'b0110111 : begin //LUI
            position = 0;
         end
         7'b0010111 : begin //AUIPC
            position = 1;
         end
         7'b1101111 : begin //JAL
            position = 2;
         end
         7'b1100111 : begin //JALR
            position = 3;
         end
         7'b1100011 : begin //BEQ, BNE, BLT, BGE, BLTU, BGEU
            case (funct3)
                3'b000 : begin //BEQ
                    position = 4;
                end
                3'b001 : begin //BNE
                    position = 5;
                end
                3'b100 : begin //BLT
                    position = 6;
                end
                3'b101 : begin //BGE
                    position = 7;
                end
                3'b110 : begin //BLTU
                    position = 8;
                end
                3'b111 : begin //BGEU
                    position = 9;
                end
                default : begin
                end/* Unknown Instruction */
            endcase // funct3
         end
         7'b0000011 : begin //LB, LH, LW, LBU, LHU
            case (funct3)
                3'b000 : begin //LB
                    position = 10;
                end
                3'b001 : begin //LH
                    position = 11;
                end
                3'b010 : begin //LW
                    position = 12;
                end
                3'b100 : begin //LBU
                    position = 13;
                end
                3'b101 : begin //LHU
                    position = 14;
                end
                default : begin
                end/* Unknown Instruction */
            endcase // funct3
         end
         7'b0100011 : begin //SB, SH, SW
            case (funct3)
                3'b000 : begin //SB
                    position = 15;
                end
                3'b001 : begin //SH
                    position = 16;
                end
                3'b010 : begin //SW
                    position = 17;
                end
                default : begin
                end/* Unknown Instruction */
            endcase
         end
         7'b0010011 : begin //ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
            case (funct3)
                3'b000 : begin //ADDI
                    position = 18;
                end
                3'b010 : begin //SLTI
                    position = 19;
                end
                3'b011 : begin //SLTIU
                    position = 20;
                end
                3'b100 : begin //XORI
                    position = 21;
                end
                3'b110 : begin //ORI
                    position = 22;
                end
                3'b111 : begin //ANDI
                    position = 23;
                end
                3'b001 : begin //SLLI
                    position = 24;
                end
                3'b101 : begin //SRLI, SRAI
                    if(local_inst[31:25] == 7'b0000000) begin
                        position = 25;
                    end
                    else if(local_inst[31:25] == 7'b0100000) begin
                        position = 26;
                    end
                end
                default : begin
                end/* Unknown Instruction */
            endcase
         end
         7'b0110011 : begin //ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
            case (funct3)
                3'b000 : begin //ADD, SUB
                    if(local_inst[31:25] == 7'b0000000) begin
                        position = 27;
                    end
                    else if(local_inst[31:25] == 7'b0100000) begin
                        position = 28;
                    end
                end
                3'b001 : begin //SLL
                    position = 29;
                end
                3'b010 : begin //SLT
                    position = 30;
                end
                3'b011 : begin //SLTU
                    position = 31;
                end
                3'b100 : begin //XOR
                    position = 32;
                end
                3'b101 : begin //SRL, SRA
                    if(local_inst[31:25] == 7'b0000000) begin
                        position = 33;
                    end
                    else if(local_inst[31:25] == 7'b0100000) begin
                        position = 34;
                    end
                end
                3'b110 : begin //OR
                    position = 35;
                end
                3'b111 : begin //AND
                    position = 36;
                end
                default : begin
                end/* Unknown Instruction */
            endcase
         end
         7'b0001111 : begin //FENCE, FENCE.I
            case (funct3)
                3'b000 : begin //FENCE
                    position = 37;
                end
                3'b001 : begin //FENCE.I
                    position = 38;
                end
                default : begin
                end/* Unknown Instruction */
            endcase // funct3
         end
         7'b1110011 : begin //ECALL, EBREAK, CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI
            case (funct3)
                3'b000 : begin //ECALL, EBREAK
                    position = 39;
                    //position = 40; not implemented
                end
                3'b001 : begin //CSRRW
                    position = 41;
                end
                3'b010 : begin //CSRRS
                    position = 42;
                end
                3'b011 : begin //CSRRC
                    position = 43;
                end
                3'b101 : begin //CSRRWI
                    position = 44;
                end
                3'b110 : begin //CSRRSI
                    position = 45;
                end
                3'b111 : begin //CSRRCI
                    position = 46;
                end
            endcase // funct3
         end
         default : begin
         end/* Unknown Instruction */
      endcase

endtask : find_inst_in_set

endmodule
