# Сборка: src/app.html + src/core.js → index.html (один файл; ИИ-модель и ORT лежат рядом)
import os
root = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(root, 'src/app.html'), encoding='utf-8').read()
core = open(os.path.join(root, 'src/core.js'), encoding='utf-8').read()
assert '</script' not in core
out = src.replace('/*CORE*/', core)
open(os.path.join(root, 'index.html'), 'w', encoding='utf-8').write(out)
print('index.html', len(out))
