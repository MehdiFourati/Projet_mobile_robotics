program = """
onevent speed
    motor.left.target = event.args[0]
    motor.right.target = event.args[1]
"""


def controlling_wheels_speed(left_wheel,right_wheel,aw,node):

    aw(node.register_events([("speed", 2)]))
    aw(node.compile(program))
    aw(node.send_events({"speed": [left_wheel,right_wheel]}))
    aw(node.run())

    return 

def kidnapping(node):
    
    default_value = [0,0,20]
    combined_error = sum(abs(curr - default) for curr, default in zip(node.v.acc, default_value))
    print(combined_error)
    if combined_error > 10:
        return True
    else:
        return False



    



