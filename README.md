# Desarrollo de control embebido de motores
Este repositorio presenta el diseño de un sistema maestro-esclavo utilizado sobre las placas de desarrollo “Discovery”, para control de motores. 
Este se concibió con una placa maestro, la cual se encargue de coordinar los movimientos de los motores, recibir instrucciones de movimientos por medio de USB, 
y realizar, en caso de ser necesarios; los cálculos de cinemática inversa. Por su parte, cada placa esclavo se encargará de llevar a cabo el movimiento de 
dos motores (por esclavo) con su debido control. Para el desarrollo del proyecto, se hizo uso del software STM32Cube IDE.
