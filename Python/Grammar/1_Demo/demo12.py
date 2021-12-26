import turtle

colors = ("red", "green", "yellow", "black")

t = turtle.Pen()
t.width(2)
t.speed(15)
for i in range(10):
    t.penup()
    t.goto(0, -i * 10)
    t.pendown()
    t.color(colors[i % len(colors)])
    t.circle(15 + i * 10)

turtle.done()
