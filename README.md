# STM32 Room Control | CubeMX & HAL

**Academic coursework** · STM32L476RG · Embedded C · GPIO · I²C · Keypad · OLED SSD1306

This repository contains educational firmware and documentation for modular peripheral drivers and a keypad/OLED access-control example. It is not presented as a production-ready security device or as independently hardware-validated.


**Universidad Nacional de Colombia - Sede Manizales**

**Curso:** Estructuras Computacionales (4100901)

## Introducción

En esta segunda fase del curso **Estructuras Computacionales**, nos enfocaremos en la **implementación de los componentes fundamentales de un dispositivo computacional**. El propósito principal es fortalecer las habilidades en la creación y reutilización de drivers/librerías, así como en la integración de componentes de entrada/salida (I/O).

Los conocimientos adquiridos se aplicarán progresivamente en la propuesta para el proyecto final llamado **"Control de Sala"**, que también integrará:

* Un teclado Hexadecimal.
* Un display OLED para visualización.
* Un sensor de temperatura.
* Un sistema de ventilación controlado por PWM.
* Conexión a Internet mediante el módulo ESP01.

## Objetivos Específicos

* Comprender y dominar el uso de STM32CubeMX para la configuración inicial de proyectos.
* Aprender los principios y la estructura de librerías modulares en C.
* Implementar componentes como GPIO, UART, Timers y Buffers circulares.
* Integrar estos componentes para construir sistemas robustos y reutilizables.

## Alcance verificado

El código versionado incluye un ejemplo de ingreso de PIN por teclado matricial con salida en OLED SSD1306, indicador LED y buffer circular. Las características de temperatura, ventilación e Internet que se mencionan arriba corresponden a la propuesta del curso; **no se afirma que estén integradas y comprobadas en el firmware actual**.

Este es un repositorio de material académico. No se ha realizado en esta revisión una prueba física independiente.

## Documentación

Este repositorio contiene las siguientes guías para acompañar el proceso de aprendizaje:

* [`Doc/SETUP.md`](Doc/SETUP.md): Guía de configuración del entorno de desarrollo.
* [`Doc/CUBEMX_CONFIG.md`](Doc/CUBEMX_CONFIG.md): Introducción a STM32CubeMX y generación de proyectos.
* [`Doc/LIB_PRINCIPLES.md`](Doc/LIB_PRINCIPLES.md): Principios para la creación de librerías en C.
* [`Doc/RING_BUFFER.md`](Doc/RING_BUFFER.md): Implementación de una librería de buffer circular.
* [`Doc/KEYPAD.md`](Doc/KEYPAD.md): Manejo de teclado matricial.
* [`Doc/SSD1306.md`](Doc/SSD1306.md): Interfaz con pantalla OLED.

Cada guía contiene ejemplos prácticos y explicaciones orientadas a fortalecer la comprensión conceptual y la implementación práctica.

---

¡Manos a la obra! Comienza por revisar la [Configuración del Entorno (Doc/SETUP.md)](Doc/SETUP.md).
