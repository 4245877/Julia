"""Build readable render/contact evidence from the verified texture work."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT / 'docs/raphtalia/texturing'


def font(size, bold=False):
    return ImageFont.truetype('C:/Windows/Fonts/' + ('arialbd.ttf' if bold else 'arial.ttf'), size)


def fit_image(path, canvas, box, crop_alpha=True):
    image = Image.open(path).convert('RGBA')
    if crop_alpha and image.getchannel('A').getbbox():
        image = image.crop(image.getchannel('A').getbbox())
    x, y, w, h = box
    image.thumbnail((w, h), Image.Resampling.LANCZOS)
    canvas.alpha_composite(image, (x+(w-image.width)//2, y+(h-image.height)//2))


def main():
    board = Image.new('RGBA', (1880, 1480), '#101722')
    draw = ImageDraw.Draw(board)
    draw.text((60, 38), 'RAPHTALIA', font=font(46, True), fill='#f6f0e7')
    draw.text((62, 100), 'Текстуры 4K · исходная геометрия · материалы по референсу Newtaw', font=font(24), fill='#a7b5c6')
    for label, x in [('СПЕРЕДИ', 60), ('СЗАДИ', 650), ('ДЕТАЛИ', 1240)]:
        draw.text((x, 158), label, font=font(20, True), fill='#d5b778')
    for box in [(40, 200, 560, 1170), (630, 200, 560, 1170), (1220, 200, 620, 575), (1220, 800, 620, 570)]:
        x, y, w, h = box
        draw.rounded_rectangle((x, y, x+w, y+h), radius=16, fill='#1c2531')
    fit_image(DOC/'front.png', board, (65, 225, 510, 1120))
    fit_image(DOC/'back.png', board, (655, 225, 510, 1120))
    fit_image(DOC/'face.png', board, (1240, 215, 580, 550), crop_alpha=False)
    fit_image(DOC/'outfit.png', board, (1240, 815, 580, 540), crop_alpha=False)
    draw.text((60, 1410), '24 PNG × 4096²    ·    BLEND / FBX    ·    429 костей    ·    15 анимаций', font=font(24), fill='#a7b5c6')
    board.convert('RGB').save(DOC/'preview.png')
    files = sorted((ROOT/'assets/textures/raphtalia').glob('*.png'))
    contact = Image.new('RGB', (1536, 4*290), '#202530')
    draw = ImageDraw.Draw(contact)
    for i, path in enumerate(files):
        image = Image.open(path).convert('RGB')
        image.thumbnail((256, 256), Image.Resampling.LANCZOS)
        x, y = i%6*256, i//6*290
        contact.paste(image, (x, y))
        draw.text((x+8, y+260), path.stem, font=font(18), fill='white')
    contact.save(DOC/'maps_contact.png')


if __name__ == '__main__':
    main()
