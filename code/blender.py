import bpy
import time

for p in [1,2,3,4,5]:
    bpy.data.objects["Cube"].location.x  += 1.0
    bpy.ops.render.render()
    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)