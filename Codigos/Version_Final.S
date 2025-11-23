
# Test code I2C
	.section .text
	.globl _start
	.equ addr, 0x78  

_start:
 #Setup Necesario para el algoritmo. Se usan GPIO3 Y GPIO2, 3=SCL, 2=SDA. Las mascaras dependen de esto, ajustar para otros GPIO

   li s0, 0x0010C02        #Configuracion del registro x0, pone GPIO 2, 3 en 1 para recepcion
   li s9, 0x00108002

   csrrw x0 , 0, s0				#Configuracion para poner GPIO 2 en envio

   li s2, 0                     #Registro donde se almacena ADDR recibida

   li s3, 8                     #Contador de addr+w/r

   li s4, addr                  #Direccion de memoria asignada

   li s5, 0                     #Registro donde se almacenan los datos 
  
   li s7, 6					    #Delay del pulso ACK 0x8 para F 100 Khz
 
   li s8, 2                     #Contador para ack



 

 
	

_SCLstart:
	li t1, 0                    #Limpio los registros

	li t2, 0

	csrrs t1 , 4, ra            #Leo GPIOs 

 	andi t1, t1 ,0x08           #Mascara GPIO3 (SCL)	

	beqz t1, _SCLstart          #Si el valor del bit en t1 es 0, volver a muestrear

#------------------------------------------------------------------------------------

	csrrs t2, 4, ra             #Leo GPIOs

	andi t2, t2, 0x04           #Mascara GPIO2 (SDA)

	beqz  t2, wait_scl_high     #Si GPIO2 es 0, y GPIO 3 es 1, saltar a recepcion

#------------------------------------------------------------------------------------	

	bne t2, t3, _SCLstart       #Si no, saltar al inicio del loop y volver a muestrear


#-------------------------------------------------------------------------------------

wait_scl_high:
	 
	li t1, 0 			         #Limpio los registros

	li t2, 0

	csrrs t1,4,ra                #Leo GPIOs 

 	andi t1,t1,0x08              #Mascara GPIO3 (SCL)	

	beqz t1, wait_scl_high       #Si el valor del bit en t1 es 0, volver a muestrear
#------------------------------------------------------------------------------------------- 
	csrrs t2, 4, ra              #Leer Gpios

	andi t2, t2, 0x4             #Mascara GPIO2 (SDA)

	srli t2, t2, 2               #Lo muevo al bit 0 del registro t2
#----------------------------------------------------------------------------------------------	
	slli s2, s2, 1               #Desplazo un bit a la izquierda

	or   s2, s2, t2              #Pongo el valor de SDA en el bit MSB

	addi   s3, s3, -1            #Bajo el contador de addr

	bnez   s3, wait_scl_low      #si no hay 8 bits, esperar el cambio de flanco

	j addr_final


wait_scl_low:
    csrrs  t1, 4, ra             #Leo GPIO3

    andi   t1, t1, 0x08          #Mascara GPIO3 SCL

    bnez   t1, wait_scl_low      #Si el valor no es 0, volver a muestrear SCL

    j      wait_scl_high         #Saltar a buscar el siguiente valor de SDA.
	 



addr_final:
	

	#bne s4, s2, _start    		  #Si el addr recibido no es igual al asignado, iniciar programa de nuevo

	li t1, 0 				      #Limpio Registro

	csrrs  t1, 4, ra              #Leo GPIO3

    andi   t1, t1, 0x08           #Mascara GPIO3 SCL

    bnez   t1, addr_final         #Si el valor no es 0, volver a muestrear SCL

	j SendAckH



SendAckH:
	csrrs  t1, 4, ra             #Leo GPIO3

    andi   t1, t1, 0x08          #Mascara GPIO3 SCL

    bnez t1, SendAckH			 #Si T1 no es 0, volver a muestrear
    					 
    li t0, 0x4			         #Mascara para GPIO 2

	li s6, 8 					 #Contador de bits de datos

    csrrw x0 , 0, s9			 #Pongo el GPIO 2 en modo envio

    csrrw x0 , 4, t0             #Enciendo el GPIO 

    addi s8,s8,-1				 #Resto 1 al contador de ACKS

    				            



SendAckL:
	addi t1, t1, 1				  #Delay para envio del pulso de ACK

	bne s7, t1, SendAckL		 
  
    csrrw x0 , 4, ra		      #Apago el GPIO 2

    csrrw x0 , 0, s0			  #Pongo el GPIO 2 en modo recepcion
		   		  				 
    bnez s8, Data_SCL			  #Reviso el contador de ACK, si es 0, ir a terminar el programa

    j finish




Data_SCL:

	li t1, 0

	li t2, 0

	csrrs t1 , 4, ra      #Leer GPIOs 

	andi t1, t1 ,0x08     #Mascara GPIO3 SCL	

	beqz t1, Data_SCL     #Si el valor del bit en t1 es 0, volver a muestrear
#-------------------------------------------------------------------------------------------
	csrrs t2, 4, ra       #Leer Gpios

	andi t2, t2, 0x4      #Mascara GPIO2 SDA

	srli t2, t2, 2        #Lo muevo al bit 0 del registro t2

	slli s5, s5, 1        #Desplazo un bit a la izquierda

	or   s5, s5, t2       #Pongo el valor de SDA en el bit MSB
#---------------------------------------------------------------------------------------------	
	addi s6, s6,-1        #Le resto 1 al contador de bits de datos

	beqz s6, SendAckH      #Loopear hasta tener 8 .bits y mandar ack


wait_scl_low2:

	li  t1, 0

    csrrs  t1, 4, ra         #Leo GPIO3

    andi   t1, t1, 0x08      #Mascara GPIO3 SCL

    bnez   t1, wait_scl_low2  #Si el valor no es 0, volver a muestrear SCL

    j      Data_SCL         #Saltar a buscar el siguiente valor de SDA.	



finish:
	
	j finish









