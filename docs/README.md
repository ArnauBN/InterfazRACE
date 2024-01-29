Interfaz Gráfica Robot
---

Para establecer el entorno virtual de Python, desde miniconda/anaconda prompt:

`conda env create -f environment.yml`

Este entorno no incluye rospy

---

# Estructura

## Carpetas

`config\experiments`: Los archivos contenidos en esta carpeta son archivos de texto que definen los experimentos. 

`controllers`: Contiene un controlador por cada ventana de la GUI.

`docs`: Contiene cualquier archivo de documentación. Como este `README.md`.

`models`: Contiene todos los modelos de datos y dispositivos.

`resources`: Contiene todos los recursos multimedia (actualmente imágenes).
	`icons`: Contiene iconos 16x16.
	`logos`: Contiene logos RACE, UMH, UMA y UVA.
	
`styles`: Actualmente vacía. Contendría la definición de estilos personalizados si fuera necesario.

`tests`: Contiene archivos `.py` de test. Actualmente solo contiene tests para la cámara RealSense de profundidad.

`translations`: Actualmente vacía. Contendría la definición de otros idiomas.

`ui_files`: Contiene un archivo `.ui` por cada ventana. Definen la apariencia de la interfaz.

`utils`: Contiene archivos `.py` de utilidades, incluyendo variables globales.

`venv`: Contiene el archivo del entorno virtual. Formato anaconda `.yml`.

`views`: Contiene un archivo `.py` por cada ventana. Cargan los archivos `.ui` y añaden logos y algo de formato adicional.

`widgets`: Contiene widgets personalizados.



## Archivos
`experimentos`: Actualmente definen los procesos del DFD (Data-Flow-Diagram) con el siguiente formato:

`
# Experimento 1
process P1 Fase 1
process P2 Fase 2
process P3 Fase 3
`

Si la primera línea empieza con `#`, esta línea indica el nombre del experimento. La sintaxis empleada es la definida en: https://github.com/pbauermeister/dfd

