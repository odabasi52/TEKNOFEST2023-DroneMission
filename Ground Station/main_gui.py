#######GUI STUFFS
import tkinter as tk
from tkinter import ttk
from tkinter.font import Font

######MATH STUFFS
import math
from math import *

#######MAP LIBS
import tkintermapview as map
from PIL import ImageTk, Image
import numpy as np
import os

import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)

IP = "192.168.160.1"
PORT = "4141"
TCP_CONNECTION = "tcp://"+IP+":"+PORT

socket.connect(TCP_CONNECTION)
socket.setsockopt_string(zmq.SUBSCRIBE,"")

#######################CREATE MAIN WINDOW
window = tk.Tk()
######################GET CURRENT WORKING DIRECTORY
PATH = os.getcwd()

#######################SOME WIDGET CLASSES
################################################SPEEDO METER
width,height = 330,330 
len1,len2 = 0.85,0.3 
ray = int(0.7*width/2) 
x0,y0 = width/2,width/2 
meter_font = Font(family="Tahoma",size=10,weight='normal')
class Meter(tk.Canvas):
    def draw(self,vmin,vmax,step, outside_color,inside_color, needle_color, label_color):
        self.vmin = vmin
        self.vmax = vmax
        x0 = width/2
        y0 = width/2
        ray = int(0.7*width/2)

        self.create_oval(x0-ray*1.1,y0-ray*1.1,x0+ray*1.1,y0+ray*1.1,
            fill=outside_color)#The gray outer ring.
        self.create_oval(x0-ray,y0-ray,x0+ray,y0+ray,fill=inside_color)#The dial.
        coef = 0.1
        self.create_oval(x0-ray*coef,y0-ray*coef,x0+ray*coef,y0+ray*coef,
            fill=needle_color)#This is the connection point blob of the needle.
        self.unit = self.create_text(width/2,height*.7,fill=label_color,
            font=meter_font)
        #This loop fills in the values at each step or gradation of the dial.
        for i in range(1+int((vmax-vmin)/step)):
            v = vmin + step*i
            angle = (5+6*((v-vmin)/(vmax-vmin)))*math.pi/4
            self.create_line(x0+ray*math.sin(angle)*0.9,
                y0 - ray*math.cos(angle)*0.9,
                x0+ray*math.sin(angle)*0.98,
                y0 - ray*math.cos(angle)*0.98,fill=label_color,width=2)
            self.create_text(x0+ray*math.sin(angle)*0.75,
                y0 - ray*math.cos(angle)*0.75,
                text=v,fill=label_color,font=meter_font)
            if i==int(vmax-vmin)/step:
                continue
            for dv in range(1,5):
                angle = (5+6*((v+dv*(step/5)-vmin)/(vmax-vmin)))*math.pi/4
                self.create_line(x0+ray*math.sin(angle)*0.94,
                    y0 - ray*math.cos(angle)*0.94,
                    x0+ray*math.sin(angle)*0.98,
                    y0 - ray*math.cos(angle)*0.98,fill=label_color)
        self.needle = self.create_line(x0-ray*math.sin(5*math.pi/4)*len2,
            y0+ray*math.cos(5*math.pi/4)*len2,
            x0+ray*math.sin(5*math.pi/4)*len1,
            y0-ray*math.cos(5*math.pi/4)*len1,
            width=2,fill=needle_color)

    #Draws the needle based on the speed or input value.
    def draw_needle(self,v):        
        v = max(v,self.vmin)#If input is less than 0 then the pointer stays at 0
        v = min(v,self.vmax)#If input is greater than the greatest value then the pointer stays at the maximum value.
        angle = (5+6*((v-self.vmin)/(self.vmax-self.vmin)))*math.pi/4
        self.coords(self.needle,x0-ray*math.sin(angle)*len2,
            y0+ray*math.cos(angle)*len2,
            x0+ray*math.sin(angle)*len1,
            y0-ray*math.cos(angle)*len1)

################################################COMPASS
class Pusula(tk.Frame):
    wind_rose = {'N': 0.0,
                 'NNE': pi / 8,
                 'NE': pi / 4,
                 'ENE': pi*3 / 8,
                 'E': pi / 2,
                 'ESE': pi*5 / 8,
                 'SE': pi*3 / 4,
                 'SSE': pi*7 / 8,
                 'S': pi,
                 'SSW': pi*9 / 8,
                 'SW': pi*5 / 4,
                 'WSW': pi*11 / 8,
                 'W': pi*3 / 2,
                 'WNW': pi*13 / 8,
                 'NW': pi*7 / 4,
                 'NNW': pi*15 / 8}

    def __init__(self, master, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.master = master

        img_bg = Image.open('./images/compass.png')
        self.center = (img_bg.width // 2, img_bg.height // 2)
        bg = ImageTk.PhotoImage(img_bg)
        self.bg = bg  # keep reference

        self.img_disc = Image.open('./images/compass_disc.png')
        disc = ImageTk.PhotoImage(self.img_disc)
        self.disc = disc

        self.canvas = tk.Canvas(self, width=img_bg.width, height=img_bg.height)
        self.canvas.configure(state=tk.DISABLED, bg='gray25')
        self.canvas.pack()

        self.canvas.create_image(self.center, image=bg)
        self.canvas.create_image(self.center, image=disc)

    def display_compass(self, angle=0):
        self.canvas.delete(self.disc)
        img_rot = self.img_disc.rotate(degrees(angle))
        disc = ImageTk.PhotoImage(img_rot)
        self.canvas.create_image(self.center, image=disc)
        self.disc = disc  



##################CONFIGURE TABS AND WINDOW
window.title("TAY TULPAR #TEKNOFEST-2023")
window.minsize(1000,610)
window.maxsize(1000,610)
window.config(background="black")
window.wm_iconphoto(False, tk.PhotoImage(file = PATH+"/images/logo.png"))

tabController = ttk.Notebook(window, width=1000, height=610)
tab1 = tk.Frame(tabController, bg="black")
tab2 = tk.Frame(tabController, bg="#d9d9d9")
tab4 = tk.Frame(tabController, bg="black")

tabController.add(tab1, text="HARITA")
tabController.add(tab2, text="HIZ & YUKSEKLIK")
tabController.add(tab4, text="PUSULA & BATARYA")

tabController.pack()


#################TAB1
##########################################TITLE
tk.Label(tab1, background="black",height=1,text="").pack()
title = tk.Label(tab1, background="black",height=1,font=("Helvetica Bold",20), foreground="white",text="SABIT KANAT IHA GOREV TAKIBI")
title.pack()

##########################################MAP
map_label = tk.LabelFrame(tab1)
map_label.pack()
map_widget = map.TkinterMapView(map_label, width=400, height=400)
map_widget.set_position(10, 10)
marker = map_widget.set_marker(10, 10, text="BURADA")
map_widget.pack()    

##########################################ENLEM
tk.Label(tab1, background="black",height=1,text="").pack()
enlem_label = tk.Label(tab1, background="black", height=1,font=("Helvetica Bold",11), foreground="white",text="ENLEM = "+str(10)+"° Kuzey")
enlem_label.pack()

##########################################BOYLAM
boylam_label = tk.Label(tab1,background="black",height=1,font=("Helvetica Bold",11), foreground="white", text="BOYLAM = "+str(10)+"° Doğu")
boylam_label.pack()

#########################################STATUS LABEL
ucak_durum_label = tk.Label(tab1,background="black",height=1,font=("Helvetica Bold",11), foreground="white", text="UCAGIN MODU: "+str(10))
ucak_durum_label.pack()


##############TAB2
##########################################HIZ GOSTERGESI, YUKSEKLIK
alt = tk.Frame(tab2,width=width,height=width,bg="black")
altitude = Meter(alt,width=width,height=height)
altitude.draw(0,50,5,"black","gray","dark gray","black")
altitude.itemconfig(altitude.unit, text="YUKSEKLIK(m)")
altitude.pack()
alt.pack()

meters = tk.Frame(tab2,width=width,height=width,bg="black")
speed = Meter(meters,width=width,height=height)
speed.draw(0,30,2,"dark blue","black","red","white")
speed.itemconfig(speed.unit, text="HIZ(m/s)")
speed.pack()
meters.pack()

##############TAB4
##########################################PUSULA ve BATARYA
tk.Label(tab4, background="black",height=1,text="").pack()
pusula = Pusula(tab4)
pusula.pack()

style = ttk.Style()
style.theme_use('alt')
style.configure("green.Horizontal.TProgressbar",
            foreground='green', background='green')
tk.Label(tab4, background="black",height=1,text="").pack()
batarya = ttk.Progressbar(tab4, length = 100, mode="determinate", style="green.Horizontal.TProgressbar")
batarya.pack()
charge = tk.Label(tab4, background="black",foreground="white",height=1,text="%0")
charge.pack()
image = Image.open(PATH+"/images/tulpar.png")
image = ImageTk.PhotoImage(image)
image_label = tk.Label(tab4, image=image).pack()

###############UPDATE STUFF
shown = False
def update():
    global window, enlem_label, boylam_label, marker, shown, map_widget, ucak_durum_label
    global speed
    global altitude
    global pusula, batarya, charge
    global socket, image_label

    choise_dict = socket.recv_pyobj()

    ####UPDATE LABELS
    enlem_label.config(text="ENLEM = "+str(choise_dict["enlem"])+"° Kuzey")
    boylam_label.config(text="BOYLAM = "+str(choise_dict["boylam"])+"° Doğu")
    ucak_durum_label.config(text="UCAGIN MODU: "+str(choise_dict["durum"]))

    ####UPDATE MAP
    if shown == False:
        shown = True
        map_widget.set_position(choise_dict["enlem"], choise_dict["boylam"])
    marker.set_position(choise_dict["enlem"], choise_dict["boylam"])

    ####UPDATE SPEED
    velo = math.sqrt(choise_dict["hiz_y"]**2 + choise_dict["hiz_x"]**2)
    speed.draw_needle(int(velo))

    #####UPDATE HEIGHT
    alt = choise_dict["yukseklik"]
    altitude.draw_needle(alt)

    ####UPDATE PUSULA
    pusula_angle = (((choise_dict["pusula"]))/360)*6.3
    pusula.display_compass(angle=pusula_angle)
    
    ####UPDATE BATARYA
    batarya.config(value=choise_dict["battery"])
    charge.config(text="%"+str(round(choise_dict["battery"],2)))

    window.after(10, update)


update()
window.mainloop()