import tkinter as tk

print("before tk")

root = tk.Tk()

print("after tk")

root.title("Python Viewer")

label = tk.Label(
    root,
    text="Python has started!",
    font=("Arial",20)
)

label.pack(padx=50, pady=50)

root.mainloop()

print("window closed")