# Caja de la juntada

Informe final del Trabajo Práctico Final (Parcial 2) de **Integración Tecnológica / Internet de las Cosas**.

Una caja con cuatro casilleros donde cada participante de una juntada deja el celular. Con la juntada en marcha hay luz ambiente, cronómetro y una bandera levantada por un servo. Si alguien saca el celular, la caja lo delata con luz y sonido, lo registra y al final manda un resumen a Telegram.

- **Modalidad:** simulada en Wokwi.
- **Plataforma:** ESP32 (firmware v0.6.4).
- **Informe:** `index.html` (página única, funciona sin conexión).

## Estructura

| Ruta | Contenido |
|---|---|
| `index.html` | El informe. Los datos editables (portada, Gantt, pruebas, evidencias, checklist) están en el primer `<script>` del final del archivo. |
| `codigo/sketch_v0-6-4.ino` | Firmware vigente. Las credenciales de Telegram están reemplazadas por `TU_TOKEN` y `TU_CHAT_ID`. |
| `assets/` | Fuentes tipográficas, copia descargable del firmware e imágenes de evidencia. |
| `tools/embed.py` | Vuelve a embeber las fuentes y el firmware en `index.html` después de cambiar el `.ino`. |

## Modo final

Agregar `?final` a la URL oculta los marcadores de pendiente.
