import PySimpleGUI as sg
import time
## Conventions
#   <PySimpleGui Element>_<Property>
sg.theme("SandyBeach")
## Title Element ##
title_font = ("Times New Roman",32)
title_size = 20
title_reagent = sg.Text("Reagent",size=(title_size),font=title_font)
title_flowrate = sg.Text("Flow Rate (uL/min)",size=(title_size),font=title_font)
title_volume = sg.Text("Volume (uL)",size=title_size,font=title_font)
title_status = sg.Text("Status: ",size=title_size,font=title_font)
title_current_status = sg.Text("WAITING",text_color="green",size=title_size,font=title_font,key="temp")

## Radio Element ## 
radio_font = ("Times New Roman",18)
radio_size = 15
radio_list = []
radio_key = []
num_of_reagents = 6
for i in range(num_of_reagents):
    letter = chr(65+i)
    radio_key.append(letter)
    radio_list.append(sg.Radio("Reagent "+letter,"REAGENTS",size=radio_size,font=radio_font,key=letter))

## Input Flowrate Composite Element ##
input_flowrate_font = ("Times New Roman",18)
input_flowrate_size = 15
input_flowrate_key = "FLOWRATE_ELEMENT"
input_flowrate_range = [x for x in range(1,100)]
input_flowrate_element = sg.Input(default_text="1",s=input_flowrate_size,justification="right",font=input_flowrate_font,key=input_flowrate_key)
# Up Button Element
button_flowrate_inc_image_file = "increment_left.png"
button_flowrate_inc_image_size = (40,40)
button_flowrate_inc_image_subsample = 5 
button_flowrate_color=("yellow","white")
button_flowrate_inc_key= "FLOWRATE_INCREMENT_KEY"
button_flowrate_inc_element = sg.RealtimeButton(button_text="",key=button_flowrate_inc_key,button_color=button_flowrate_color,image_filename=button_flowrate_inc_image_file,image_size=button_flowrate_inc_image_size,image_subsample=button_flowrate_inc_image_subsample)

# Down Button Element
button_flowrate_dec_image_file = "decrement_right.png"
button_flowrate_dec_image_size = (40,40)
button_flowrate_dec_image_subsample = 5 
button_flowrate_dec_key= "FLOWRATE_DECREMENT_KEY"
button_flowrate_dec_element = sg.RealtimeButton(button_text="",key=button_flowrate_dec_key,button_color=button_flowrate_color,image_filename=button_flowrate_dec_image_file,image_size=button_flowrate_dec_image_size,image_subsample=button_flowrate_dec_image_subsample)

## Input Volume Composite Element ##
# Input Element
input_volume_font = ("Times New Roman",18)
input_volume_size = 15
input_volume_key  = "VOLUME_ELEMENT"
input_volume_range = [x for x in range(1,100)]
input_volume_element =  sg.Input(default_text="1",s=input_volume_size,justification="right",font=input_volume_font,key=input_volume_key)
# Up Button Element
button_volume_inc_image_file = "increment_left.png"
button_volume_inc_image_size = (40,40)
button_volume_inc_image_subsample = 5 
button_volume_color=("yellow","white")
button_volume_inc_key= "VOLUME_INCREMENT_KEY"
button_volume_inc_element = sg.RealtimeButton(button_text="",key=button_volume_inc_key,button_color=button_volume_color,image_filename=button_volume_inc_image_file,image_size=button_volume_inc_image_size,image_subsample=button_volume_inc_image_subsample)

# Down Button Element
button_volume_dec_image_file = "decrement_right.png"
button_volume_dec_image_size = (40,40)
button_volume_dec_image_subsample = 5 
button_volume_dec_key= "VOLUMNE_DECREMENT_KEY"
button_volume_dec_element = sg.RealtimeButton(button_text="",key=button_volume_dec_key,button_color=button_volume_color,image_filename=button_volume_dec_image_file,image_size=button_volume_dec_image_size,image_subsample=button_volume_dec_image_subsample)

## List Box Composite Element ##
# List Box
reagent_header = "\t\tReagent"
flowrate_header = "Flowrate"
volume_header = "Volume"
listbox__font = ("Times New Roman",24)
listbox_values = ["\t\t".join([reagent_header,flowrate_header,volume_header])] 
listbox_element = sg.Listbox(values=listbox_values, enable_events=True, size=(40, 24), key="test",no_scrollbar=True)

button_size = (10,2)
# Add Button
button_add_element = sg.Button("Add",size=button_size)

## Button Dispense Element ##
button_dispense_element = sg.Button("Dispense",size=button_size)



'''
table_font = ("Times New Roman",18)
table_size = 15
headings = ['Regeant', 'Flow Rate (uL/min)', 'Volume (uL)']  # the text of the headings
header =  [[sg.Text('  ')] + [sg.Text(h, size=(14,1)) for h in headings]]  # build header layout

data = [["Reagent A","32 uL/min","60 uL"]]
table = sg.Table(data,headings=header)
'''
## Column 1 Layout ##
column_1 = [
    [title_reagent],
    radio_list[:3],
    radio_list[3:],
    [title_flowrate,button_flowrate_inc_element, input_flowrate_element, button_flowrate_dec_element],
    [title_volume,button_volume_inc_element,input_volume_element,button_volume_dec_element],
    [title_status,title_current_status]
]

## Column 2 Layout
column_2 = [
    [listbox_element],
    [button_add_element,button_dispense_element]
]

## Master Layout
layout = [
    [
        sg.Column(column_1),
        sg.VSeperator(),
        sg.Column(column_2)
    ]
]

## Window Init & Tuning ## 
layout_font  = ("Times New Roman",24)
window = sg.Window(title="Reagent Dispenser", layout=layout,font=layout_font, margins=(100,50),finalize=True)

## Main Code ##
flowrate_index = 0
volume_index = 0
while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        break
    elif event == "Add":
        for item in radio_key:
            if (values[item] == True):
                reagent_sel=item
                flowrate_sel = (values[input_flowrate_key]+" uL/min")
                volume_sel = (values[input_volume_key]+" uL")
                listbox_values.append('\t\t'.join([str(len(listbox_values)),reagent_sel,flowrate_sel,volume_sel]))
                break
        window["test"].update(listbox_values)
    elif event == "Dispense":
        window["temp"].update("DISPENSING",text_color="red")

    elif event == button_flowrate_inc_key:
        if(flowrate_index < len(input_flowrate_range)-1):
            flowrate_index += 1
        window[input_flowrate_key].update(input_flowrate_range[flowrate_index])
        time.sleep(0.15)
    elif event == button_flowrate_dec_key:
        if(flowrate_index > 0):
            flowrate_index -= 1
        window[input_flowrate_key].update(input_flowrate_range[flowrate_index])
        time.sleep(0.15)
    elif event == button_volume_inc_key:
        if(volume_index < len(input_volume_range)-1):
            volume_index += 1
        window[input_volume_key].update(input_volume_range[volume_index])
        time.sleep(0.15)
    elif event == button_volume_dec_key:
        if(volume_index > 0):
            volume_index -= 1
        window[input_volume_key].update(input_volume_range[volume_index])
        time.sleep(0.15)
## Close Window ##
window.close()
