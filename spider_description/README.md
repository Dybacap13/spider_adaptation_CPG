**_spider_description_**

Хранит модель робота-гексапода и его конфиги.

 * xacro - не дублирует meshes ( расчет представлен в spider.xacro)
 * urdf - дублирует meshes ( не рекомендую использовать )

В данный момент самые важные и необходимые файлы

**_spider_description_xacro_**

Расчитанная и правильно написанная модель, учтены физю особенности симулятора Gazebo

  /spider_description/urdf/spider.xacro

**_spider_params_**

Конструктивные особенности робота-гексапода ( необходимо рекдактировать при измененении модели )

  /spider_description/params/spider.yaml


Чтобы оторвать лапу роботу необходимо:

1. Зайти в файл     /spider_description/urdf/spider.xacro
2. Закоммитить нужную строчку
   
```
<xacro:leg side="r" position="r" x="-0.170" y="-0.110" angle="-${pi*3/4}" axis_femur="-1" axis_tibia="1" axis_coxa="1" mass="0.0207333939617734"/>
<xacro:leg side="r" position="m" x="0.0" y="-0.1325" angle="-${pi/2}" axis_femur="-1" axis_tibia="1" axis_coxa="1" mass="0.0207333939617734"/>
<xacro:leg side="r" position="f" x="0.1778" y="-0.110" angle="-${pi/4}" axis_femur="-1" axis_tibia="1" axis_coxa="1" mass="0.04073339396177341"/>

<xacro:leg side="l" position="r" x="-0.170" y="0.110" angle="${pi*3/4}" axis_femur="1" axis_tibia="-1" axis_coxa="1" mass="0.0207333939617734"/>
<xacro:leg side="l" position="m" x="0.0" y="0.1325" angle="${pi/2}" axis_femur="1" axis_tibia="-1" axis_coxa="1" mass="0.0207333939617734"/>
<xacro:leg side="l" position="f" x="0.170" y="0.110" angle="${pi/4}" axis_femur="1" axis_tibia="-1" axis_coxa="1" mass="0.04073339396177341"/>
```

