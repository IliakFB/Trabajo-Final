    .section .text
    .globl _start

_start:
    # Cargar constantes
    li      t0, 5          # t0 = 5
    li      t1, 3          # t1 = 3

    # Suma
    add     t2, t0, t1     # t2 = t0 + t1 = 8

    # Resta el segundo operando
    sub     t3, t2, t1     # t3 = t2 - t1 = 5

    # Bucle infinito para que no siga ejecutando basura
end:
    j       end
