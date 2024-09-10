# for cmd args
import sys
# for bmp file
from struct import pack

if len(sys.argv) < 3:
  print("error: no path args provided")
  print("mcm2bmp.py <mcm path> <bmp path>")
  exit(0)

class Bitmap():
  def __init__(s, width, height):
    s._bfType = 19778 # Bitmap signature
    s._bfReserved1 = 0
    s._bfReserved2 = 0
    s._bcPlanes = 1
    s._bcSize = 12
    s._bcBitCount = 24
    s._bfOffBits = 26
    s._bcWidth = width
    s._bcHeight = height
    s._bfSize = 26+s._bcWidth*3*s._bcHeight
    s.clear()


  def clear(s):
    s._graphics = [(255,0,255)]*s._bcWidth*s._bcHeight

  # color rgba
  def setPixel(s, x, y, color):
    if isinstance(color, tuple):
      if x<0 or y<0 or x>s._bcWidth-1 or y>s._bcHeight-1:
        print("x: ", x , "/", s._bcWidth-1, "  y: ", y, "/", s._bcHeight-1)
        #raise ValueError('Coords out of range')
        return
      if len(color) != 4:
        raise ValueError('Color must be a tuple of 4 elems')
      s._graphics[y*s._bcWidth+x] = (color[2],color[1], color[0])
    else:
      raise ValueError('Color must be a tuple of 4 elems')


  def write(s, file):
    with open(file, 'wb') as f:
      f.write(pack('<HLHHL', 
                   s._bfType, 
                   s._bfSize, 
                   s._bfReserved1, 
                   s._bfReserved2, 
                   s._bfOffBits)) # Writing BITMAPFILEHEADER
      f.write(pack('<LHHHH', 
                   s._bcSize, 
                   s._bcWidth, 
                   s._bcHeight, 
                   s._bcPlanes, 
                   s._bcBitCount)) # Writing BITMAPINFO
      for px in s._graphics:
        f.write(pack('<BBB', *px))
      for i in range((4 - ((s._bcWidth*3) % 4)) % 4):
        f.write(pack('B', 0))

'''
def main():
  side = 520
  b = Bitmap(side, side)
  for j in range(0, side):
    b.setPixel(j, j, (255, 0, 0))
    b.setPixel(j, side-j-1, (255, 0, 0))
    b.setPixel(j, 0, (255, 0, 0))
    b.setPixel(j, side-1, (255, 0, 0))
    b.setPixel(0, j, (255, 0, 0))
    b.setPixel(side-1, j, (255, 0, 0))
  b.write('file.bmp')
'''


class Font():
  def __init__(s):
    # default font file name
    s._font_file = "default"
    # array of arry of image bytes ready to upload to fc
    s._characters_bytes = []
    # array of array of image bits by character
    s._characters = []
    # an array of base64 encoded image strings by character
    s._character_image_urls = []

    # NVM ram size for one font char, actual character bytes
    s._MAX_CHAR_COUNT = 54
    # NVM ram field size for one font char, last 10 bytes dont matter **/
    s._MAX_NVM_FONT_CHAR_FIELD_SIZE = 64
    s._CHAR_HEIGHT = 18
    s._CHAR_WIDTH = 12
    s._COLORS = []
    s._COLORS.append((0,0,0,255))       #0
    s._COLORS.append((255,0,255,0))   #1
    s._COLORS.append((255,255,255,255)) #2

    # 64 x 64 char image
    s._max_bmp_chars_width = 32
    s._max_bmp_chars_height = 9

    # add one empty pixel around the chars
    s._char_padding = 1

    s._bmp = Bitmap(
      (s._max_bmp_chars_width * s._CHAR_WIDTH) + (s._max_bmp_chars_width * s._char_padding * 2), 
      (s._max_bmp_chars_height * s._CHAR_HEIGHT) + (s._max_bmp_chars_height * s._char_padding * 2)
    )

  def pushChar(s, fontCharacterBytes, fontCharacterBits):
    if len(fontCharacterBytes) != s._MAX_NVM_FONT_CHAR_FIELD_SIZE:
        return

    s._characters_bytes.append(fontCharacterBytes)
    s._characters.append(fontCharacterBits)
    s.draw(len(s._characters) - 1)

  def draw(s, charAddress):
    pixelSize = 1
    width = pixelSize * s._CHAR_WIDTH
    height = pixelSize * s._CHAR_HEIGHT

    for y in range(0, height):
      for x in range(0, width):
        if charAddress >= len(s._characters):
           print('charAddress ', charAddress ,' is not in ', len(s._characters))
        
        v = s._characters[charAddress][(y * width) + x]
        color = s._COLORS[v]

        x_char = (charAddress % s._max_bmp_chars_width)
        y_char = int(charAddress / s._max_bmp_chars_width)

        x_offset = (x_char * width) + (x_char * s._char_padding * 2) + s._char_padding
        y_offset = (y_char * height) + (y_char * s._char_padding * 2) + s._char_padding

        s._bmp.setPixel(x_offset + x, y_offset + y, color)

  def parseMCMFont(s, file):
    print("opening " + file + " ...")
    #data = dataFontFile.trim().split("\n");
    mcmfile = open(sys.argv[1], 'r')
    mcmLines = mcmfile.readlines()

    # clear local data
    s._characters = []
    s._characters_bytes = []
    s._character_image_urls = []

    # make sure the font file is valid
    if (len(mcmLines) < 1) and (mcmLines[0].trim() != 'MAX7456') :
      print('that font file doesnt have the MAX7456 header, giving up')
      return
    
    characterBits = []
    characterBytes = []
    # hexstring is for debugging
    #FONT.data.hexstring = [];
    for i in range(1, len(mcmLines)):
      line = mcmLines[i]
      # hexstring is for debugging
      #FONT.data.hexstring.push(`0x${parseInt(line, 2).toString(16)}`);
      # every 64 bytes (line) is a char, we're counting chars though, which are 2 bits
      if len(characterBits) == (s._MAX_NVM_FONT_CHAR_FIELD_SIZE * (8 / 2)):
        s.pushChar(characterBytes, characterBits)
        characterBits = []
        characterBytes = []

      y = 0
      while y < 8:
        v = int(line[y : y + 2], 2)
        characterBits.append(v)
        y = y + 2
      
      characterBytes.append(int(line, 2))
    
    # push the last char
    s.pushChar(characterBytes, characterBits)

  def write(s, file):
    s._bmp.write(file)


font = Font()
font.parseMCMFont(sys.argv[1])
font.write(sys.argv[2])




