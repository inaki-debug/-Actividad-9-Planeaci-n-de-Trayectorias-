# -Actividad-9-Planeaci-n-de-Trayectorias-

En esta actividad se hizo una planeación de una trayectoria a partir de una imágen. El objetivo es dibujar el contorno de la imágen de un felino con la trayectoria del robot. Para lograrlo, se extrajeron las coordenadas del contorno del felino para convertiros a una lista de waypoints que el robot debe de seguir en orden.

Se realizó un script en MATLAB el cual recibe la imágen y la coloca en un plano cartesiano normalizado de 12x12 para que la trayectoria no sea muy larga para el robot. Este script tiene las siguientes caracteristicas:

- Debido a que las imágenes tienen su origen en la esquina superior izquierda, se utilizó la función `flipud` para invertir la matriz de la imagen y alinear los sistemas de referencia.
- La captura de las coordenadas se realiza mediante click en cualquier punto de la imágen. Este click se registra con la función `ginput` y la coordenada se va almacenando en dos listas `xlist` y `ylist` para cada eje. Para finalizar la captura simplemente se presiona "Enter" para imprimir la lista de los waypoints en la consola con el siguiente formato: `waypoints = [3.8,1.7; 3.9,2; 4,2.4;...]`
- Las coordenadas son redondeadas a un decimal.

Finalmente, la captura de los waypoints tivo el siguiente resultado:

```
waypoints = [3.8,1.7; 3.9,2; 4,2.4; 4.3,2.6; 4.8,3.1; 4.8,3.3; 5,3.7; 5.1,3.8; 5.2,4.1; 5.3,4.2; 5.8,4.3; 5.9,4.5; 5.7,4.8; 5.6,5; 5.8,5.1; 5.9,5.4; 6.1,5.5; 6.3,5.8; 6.4,6.1; 6.5,6.4; 6.5,6.8; 6.5,7; 6.5,7.3; 6.5,7.5; 6.5,7.7; 6.6,8; 6.7,8.4; 6.7,8.6; 6.6,8.8; 6.8,9; 7,9.5; 7.1,9.8; 7.1,10; 7.1,10.3; 7,10.5; 6.7,10.6; 6.4,10.6; 6.1,10.5; 6,10.3; 5.7,10; 5.5,10.3; 5.4,10.5; 5.3,10.6; 4.9,10.7; 4.7,10.7; 4.5,10.7; 4.3,10.6; 4.1,10.5; 4.1,10.8; 4,11; 3.9,11.1; 3.6,11.1; 3.4,11.1; 3.1,11; 3.3,10.6; 3.1,10.9; 2.9,10.9; 2.7,10.8; 2.3,10.7; 2,10.5; 1.8,10.3; 1.6,10.1; 1.5,10.3; 1.3,10.4; 1.1,10.5; 0.9,10.5; 0.7,10.5; 0.5,10.3; 0.5,10.1; 0.5,10; 0.5,9.8; 0.5,9.6; 0.6,9.2; 0.8,9; 0.9,8.9; 0.8,8.6; 1,8; 1,7.6; 1,7.2; 1,6.8; 0.9,6.5; 1.2,6; 1.4,5.5; 1.7,5; 1.6,4.4; 1.6,4.4; 2,4.5; 2,4.5; 2.3,4.4; 2.4,4.2; 2.7,3.8; 2.8,3.5; 2.9,3.2; 3.1,2.9; 3.5,2.4; 3.6,2.2; 3.7,1.8; 3.2,3.2; 3.4,3.5; 3.6,3.6; 3.9,3.5; 4.6,3.4; 4.1,3.5; 3.8,3.7; 3.9,4.1; 4.2,4.5; 4.3,4.7; 4.5,4.9; 4,4.3; 3.8,4.1; 3.6,4.3; 3.1,4.9; 2.9,5.2; 4.7,6.8; 4.9,7; 5.4,7.1; 5.6,7.2; 5.7,7.5; 5.2,7.8; 5.3,8.2; 5,8.7; 4.8,8.7; 4.2,8; 4.1,7.8; 3.9,7.6; 3.6,7.6; 3.3,8; 3.2,8.2; 3.1,8.4; 2.8,8.7; 2.1,8.4; 2.2,8; 2.3,7.8; 1.9,7.7; 1.9,7.2; 2.3,7; 2.7,6.7];

```

Estos waypoints fueron utilizados en otro script de MATLAB el cual simula un robot con un controlador Pure Pursuit. Este controlador primer define las dimensiones del robot con un radio de 0.1m de rueda y una distancia de 0.5m entre estas. Los parámetros del controlador fueron:

```
LookaheadDistance = 0.3
DesiredLinearVelocity = 0.4
MaxAngularVelocity = 100
```

Este controlador genera las velocidades lineal y angilar dependiendo de la posición actual y al punto que quiere llegar. Usa la función `inverseKinematics` y traduce estas velocidades a la velocidad que debe llevar cada rueda. La pose del robot se calcula en cada ciclo integrando la velocidad con respecto al tiempo de muestreo que fue de 0.1s.

Este fue el resultado de la trayectoria comparada a la imágen original.

<img width="1402" height="1122" alt="felino" src="https://github.com/user-attachments/assets/f0b45dce-d360-41f6-a7de-933ec19bd222" />


<img width="1059" height="674" alt="Screenshot from 2026-05-28 22-49-15" src="https://github.com/user-attachments/assets/2c6ce836-aa6d-4ba5-852b-9e087509fb8a" />

Al final se prefirió utilizar el controlador Pure Pursuit que uno prorcional o de lazo abierto debido a que este tipo de controlador no fuerza al robot a hacer giros bruzcos debido a que respeta su física. Ademas, el `LookaheadDistance` anticipa los movimientos mientras que el controlador proporcional reacciona al error inmediato. Por último, este controlador elimina las oscilaciones ya que el proporcional al carecer de la parte derivativa, existen sobreimpulsos que provoca la ganancia P. 
