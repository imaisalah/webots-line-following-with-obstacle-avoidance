from controller import Robot

def run_robot():
    # Initialize the Webots Robot object
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    # ─── Constant Velocity & Threshold Configurations ────────────────────────
    BASE_SPEED     = 2.8    # Default cruising speed for forward motion
    TURN_SPEED     = 2.0    # Rotational speed used for spot turns
    LINE_THRESHOLD = 500.0  # IR reflection limit to differentiate line (<500)
    FRONT_DETECT   = 200.0 # Proximity sensor range to detect objects early

    # ─── Actuators Setup (Motors) ────────────────────────────────────────────
    left_motor  = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))   # Configure to velocity control mode
    right_motor.setPosition(float('inf'))

    # ─── Ground Sensors Setup (Line Tracking) ─────────────────────────────────
    gs = [robot.getDevice(f'gs{i}') for i in range(3)]
    for s in gs: 
        s.enable(time_step)

    # ─── Proximity Sensors Setup (Obstacle Detection) ────────────────────────
    ps = [robot.getDevice(f'ps{i}') for i in range(8)]
    for s in ps: 
        s.enable(time_step)

    # ─── State Machine Step Timers (Empirically Validated) ───────────────────
    STEPS_90    = 20   # Time steps required to complete a 90-degree spot turn
    STEPS_SIDE  = 35   # Steps to move sideways to clear obstacle width
    STEPS_PAST  = 55   # Steps to move forward to clear obstacle length
    IGNORE_LINE = 90   # Blinding countdown to ignore line tracks during bypass

    # ─── Initial Navigation State Variables ──────────────────────────────────
    mode         = "LINE"   # Operational state driver
    avoid_side   = 1        # Direction multiplier: +1 for Right, -1 for Left
    timer        = 0        # Tick counter for discrete motion sequences
    ignore_timer = 0        # Countdown latch to blind ground sensors temporary
    last_side    = 1        # Memory tracking of line's last registered side (+1/-1)
    search_timer = 0        # State timer for the track recovery routine

    # ─── Main Simulation Control Loop ────────────────────────────────────────
    while robot.step(time_step) != -1:

        # Acquire real-time sensor array values
        gs_v = [s.getValue() for s in gs]
        ps_v = [s.getValue() for s in ps]

        # Evaluate if individual ground sensors intersect the black track
        left_on   = gs_v[0] < LINE_THRESHOLD
        center_on = gs_v[1] < LINE_THRESHOLD
        right_on  = gs_v[2] < LINE_THRESHOLD
        on_line   = left_on or center_on or right_on

        # Store line position history right before losing tracking contact
        if on_line:
            if left_on and not right_on:
                last_side = -1  # Line shifted toward the left side
            elif right_on and not left_on:
                last_side = 1   # Line shifted toward the right side

        # Read core front-facing proximity sensors (ps0 and ps7)
        front = ps_v[0] > FRONT_DETECT or ps_v[7] > FRONT_DETECT

        # Handle sensor blinding countdown latch
        if ignore_timer > 0:
            ignore_timer -= 1

        l_speed = 0.0
        r_speed = 0.0

        # ══════════════════════ STATE 1: LINE FOLLOWING ══════════════════════
        if mode == "LINE":
            if front:
                # Obstacle detected: Evaluate optimal avoidance direction
                avoid_side   = -1 if ps_v[0] > ps_v[7] else 1
                ignore_timer = IGNORE_LINE
                timer        = STEPS_90
                mode         = "ROT_OUT"
            else:
                # Proportional logic for normal differential track following
                if center_on:
                    l_speed = BASE_SPEED
                    r_speed = BASE_SPEED
                elif left_on:
                    l_speed = BASE_SPEED * 0.05
                    r_speed = BASE_SPEED
                elif right_on:
                    l_speed = BASE_SPEED
                    r_speed = BASE_SPEED * 0.05
                else:
                    # Switch to recovery search if line contact is entirely lost
                    mode = "SEARCH"
                    search_timer = 0

        # ══════════════════════ STATE 2: ROTATE OUTWARD ══════════════════════
        elif mode == "ROT_OUT":
            # Perform sharp spot rotation away from obstacle vector
            l_speed =  TURN_SPEED * avoid_side
            r_speed = -TURN_SPEED * avoid_side
            timer  -= 1
            if timer <= 0:
                timer = STEPS_SIDE
                mode  = "FWD_SIDE"

        # ══════════════════════ STATE 3: MOVE SIDEWAYS ═══════════════════════
        elif mode == "FWD_SIDE":
            # Intercept secondary blockages safely during escape path
            if front:
                timer = STEPS_90
                mode  = "ROT_OUT"
            else:
                # Cruise forward to establish clear lateral safety margin
                l_speed = BASE_SPEED
                r_speed = BASE_SPEED
                timer  -= 1
                if timer <= 0:
                    timer = STEPS_90
                    mode  = "ROT_IN"

        # ══════════════════════ STATE 4: ROTATE INWARD ═══════════════════════
        elif mode == "ROT_IN":
            # Counter-rotate 90 degrees to align parallel with obstacle length
            l_speed = -TURN_SPEED * avoid_side
            r_speed =  TURN_SPEED * avoid_side
            timer  -= 1
            if timer <= 0:
                timer = STEPS_PAST
                mode  = "FWD_PAST"

        # ══════════════════════ STATE 5: MOVE PAST OBSTACLE ══════════════════
        elif mode == "FWD_PAST":
            # Detect secondary cascaded obstacles along the linear bypass run
            if front:
                timer = STEPS_90
                mode  = "ROT_OUT2"
            else:
                # Drive forward to fully clear the longitudinal block length
                l_speed = BASE_SPEED
                r_speed = BASE_SPEED
                timer  -= 1
                if timer <= 0:
                    timer = STEPS_90
                    mode  = "ROT_BACK"

        # ══════════════════════ STATE 6: SECONDARY ROTATION OUT ══════════════
        elif mode == "ROT_OUT2":
            l_speed =  TURN_SPEED * avoid_side
            r_speed = -TURN_SPEED * avoid_side
            timer  -= 1
            if timer <= 0:
                timer = STEPS_SIDE
                mode  = "FWD_SIDE2"

        # ══════════════════════ STATE 7: SECONDARY SIDEWAYS RUN ══════════════
        elif mode == "FWD_SIDE2":
            if front:
                timer = STEPS_90
                mode  = "ROT_OUT2"
            else:
                l_speed = BASE_SPEED
                r_speed = BASE_SPEED
                timer  -= 1
                if timer <= 0:
                    timer = STEPS_90
                    mode  = "ROT_IN"

        # ══════════════════════ STATE 8: ROTATE BACK TO TRACK ════════════════
        elif mode == "ROT_BACK":
            # Pivot inwards 90 degrees directing trajectory back toward track
            l_speed = -TURN_SPEED * avoid_side
            r_speed =  TURN_SPEED * avoid_side
            timer  -= 1
            if timer <= 0:
                mode = "RETURN"

        # ══════════════════════ STATE 9: INTERCEPT LINE RUN ══════════════════
        elif mode == "RETURN":
            # Drive forward until tracking sensors cross the original path line
            if ignore_timer == 0 and on_line:
                mode = "LINE"
            else:
                # Re-route path loop if an unforeseen blocker cuts the exit run
                if front:
                    avoid_side   = -1 if ps_v[0] > ps_v[7] else 1
                    ignore_timer = IGNORE_LINE
                    timer        = STEPS_90
                    mode         = "ROT_OUT"
                else:
                    l_speed = BASE_SPEED
                    r_speed = BASE_SPEED

        # ══════════════════════ STATE 10: LINE RECOVERY & SEARCH ═════════════
        elif mode == "SEARCH":
            if on_line:
                mode = "LINE"
            else:
                search_timer += 1
                
                # Arena perimeter boundary wall crash prevention handler
                if front:
                    # Negative counter locks system into dedicated spot-turn evasion loop
                    search_timer = -40  
                    last_side   *= -1   # Reverse scanning heading away from perimeter
                
                # Navigation motion routing inside search mode
                if search_timer < 0:
                    # Execute active 180-degree turn sequence away from wall
                    l_speed =  TURN_SPEED * last_side
                    r_speed = -TURN_SPEED * last_side
                elif search_timer <= 12:
                    # Phase 1: Micro-adjust trajectory orientation toward memory angle
                    l_speed =  TURN_SPEED * last_side
                    r_speed = -TURN_SPEED * last_side
                elif search_timer <= 140:
                    # Phase 2: High-speed linear drive across open floor to cross lost path
                    l_speed = BASE_SPEED
                    r_speed = BASE_SPEED
                else:
                    # Search window timeout: Invert search vector direction and scan next sector
                    search_timer = 0
                    last_side   *= -1 

        # Forward final computed velocities directly to wheel actuators
        left_motor.setVelocity(l_speed)
        right_motor.setVelocity(r_speed)

if __name__ == "__main__":
    run_robot()