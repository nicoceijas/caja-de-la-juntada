"""Embebe las fuentes y el firmware en index.html.

Uso (desde la carpeta del proyecto):
    python3 tools/embed.py

- Reemplaza el contenido entre /*FUENTES*/ y /*/FUENTES*/ por las @font-face con las
  fuentes de assets/fonts en base64.
- Reemplaza el contenido entre /*INO*/ y /*/INO*/ por el firmware de codigo/sketch_v0-6-4.ino
  (también lo copia a assets/ para el botón de descarga).
"""
import base64, pathlib, re, shutil

RAIZ = pathlib.Path(__file__).resolve().parent.parent
HTML = RAIZ / "index.html"
INO = RAIZ / "codigo" / "sketch_v0-6-4.ino"
FUENTES = [
    ("Fredoka", "normal", "500 600", "fredoka-var.woff2"),
    ("Atkinson Hyperlegible", "normal", "400", "atkinson-400.woff2"),
    ("Atkinson Hyperlegible", "italic", "400", "atkinson-400i.woff2"),
    ("Atkinson Hyperlegible", "normal", "700", "atkinson-700.woff2"),
    ("JetBrains Mono", "normal", "400 600", "jetbrainsmono-var.woff2"),
]

def css_fuentes():
    partes = []
    for fam, estilo, peso, archivo in FUENTES:
        b64 = base64.b64encode((RAIZ / "assets" / "fonts" / archivo).read_bytes()).decode()
        partes.append(f"@font-face{{font-family:'{fam}';font-style:{estilo};font-weight:{peso};font-display:swap;"
                      f"src:url(data:font/woff2;base64,{b64}) format('woff2')}}")
    return "\n".join(partes)

def reemplazar(texto, ini, fin, nuevo):
    patron = re.compile(re.escape(ini) + r".*?" + re.escape(fin), re.S)
    assert patron.search(texto), f"No se encontró el marcador {ini}"
    return patron.sub(lambda m: ini + nuevo + fin, texto, count=1)

def main():
    html = HTML.read_text(encoding="utf-8")
    ino = INO.read_text(encoding="utf-8")
    assert "</script" not in ino.lower()
    for clave in ("TELEGRAM_TOKEN", "TELEGRAM_CHAT_ID"):
        linea = next(l for l in ino.splitlines() if l.startswith(f"#define {clave}"))
        assert '"TU_TOKEN"' in linea or '"TU_CHAT_ID"' in linea, f"¡{clave} tiene un valor real! No se embebe."
    html = reemplazar(html, "/*FUENTES*/", "/*/FUENTES*/", "\n" + css_fuentes() + "\n")
    html = reemplazar(html, "/*INO*/", "/*/INO*/", "\n" + ino + "\n")
    HTML.write_text(html, encoding="utf-8")
    shutil.copyfile(INO, RAIZ / "assets" / "sketch_v0-6-4.ino")

    print(f"index.html: {len(html)//1024} KB")

if __name__ == "__main__":
    main()
