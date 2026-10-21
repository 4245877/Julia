"""Compare actual pre/post re-audit renders and matching measured poses."""
from pathlib import Path
import json
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'docs/raphtalia/reaudit'

def main():
    before=json.loads((OUT/'before.json').read_text());after=json.loads((OUT/'after.json').read_text())
    tests=[('elbow_-120','Локти: −120°'),('knee_120','Колени: 120°'),('ankle_35','Голеностопы: +35°'),
           ('finger_curl_30','Фаланги: +30° по поперечным осям'),('shoulder_raise_100','Плечи: 100°, стресс без подъёма ключиц'),('combined','Комбинированные движения')]
    font=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',19)
    canvas=Image.new('RGB',(1000,len(tests)*555+50),(30,32,36));draw=ImageDraw.Draw(canvas)
    draw.text((12,8),'До повторного аудита: 239 костей                 После: 241 кость, локальные исправления',font=font,fill='white')
    for row,(name,title) in enumerate(tests):
        y=50+row*555;draw.text((12,y),title,font=font,fill='white')
        for col,(prefix,data) in enumerate([('before',before),('after',after)]):
            img=Image.open(OUT/(prefix+'_'+name+'.png')).convert('RGB');img.thumbnail((500,500))
            canvas.paste(img,(col*500,y+28));m=data['poses'][name]
            draw.text((col*500+10,y+526),f"Макс. ребро {m['max_ratio']:.3f}×; рёбер >2×: {m['over_2']}",font=font,fill='white')
    canvas.save(OUT/'comparison.png');print(OUT/'comparison.png')

if __name__=='__main__':main()
