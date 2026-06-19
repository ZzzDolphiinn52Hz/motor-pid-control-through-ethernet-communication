import unicodedata

with open('Baocao.tex', 'r', encoding='utf-8') as f:
    content = f.read()

# Normalize to precomposed NFC format
normalized_content = unicodedata.normalize('NFC', content)

with open('Baocao.tex', 'w', encoding='utf-8') as f:
    f.write(normalized_content)
