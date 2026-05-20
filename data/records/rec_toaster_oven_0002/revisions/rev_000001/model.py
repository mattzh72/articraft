from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

OVEN_WIDTH = 0.56
OVEN_DEPTH = 0.44
BODY_HEIGHT = 0.35
FOOT_HEIGHT = 0.014
SHEET_THICKNESS = 0.016
LINER_THICKNESS = 0.006

LEFT_X = -OVEN_WIDTH / 2.0
RIGHT_X = OVEN_WIDTH / 2.0
FRONT_Y = -OVEN_DEPTH / 2.0
REAR_Y = OVEN_DEPTH / 2.0

DOOR_CENTER_X = -0.086
DOOR_WIDTH = 0.34
DOOR_HEIGHT = 0.248
DOOR_THICKNESS = 0.022
DOOR_HINGE_Y = FRONT_Y - DOOR_THICKNESS / 2.0
DOOR_HINGE_Z = 0.052

FASCIA_WIDTH = 0.145
FASCIA_CENTER_X = RIGHT_X - FASCIA_WIDTH / 2.0

CAVITY_WIDTH = 0.334
CAVITY_DEPTH = 0.302
CAVITY_FLOOR_Z = 0.078
CAVITY_CEILING_Z = 0.294
CAVITY_HEIGHT = CAVITY_CEILING_Z - CAVITY_FLOOR_Z
CAVITY_CENTER_Z = (CAVITY_FLOOR_Z + CAVITY_CEILING_Z) / 2.0
CAVITY_CENTER_Y = -0.055

RACK_WIDTH = 0.322
RACK_DEPTH = 0.300
RACK_GUIDE_Y = -0.020
RACK_GUIDE_Z = 0.137
RACK_CENTER_Z = 0.145

TRAY_WIDTH = 0.322
TRAY_DEPTH = 0.270
TRAY_GUIDE_Y = -0.045
TRAY_GUIDE_Z = 0.034
TRAY_CENTER_Z = 0.035


def _add_front_screw(
    part,
    *,
    x: float,
    y: float,
    z: float,
    radius: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=depth),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_side_screw(
    part,
    *,
    x: float,
    y: float,
    z: float,
    radius: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=depth),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_toaster_oven", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.26, 0.29, 0.31, 1.0))
    darker_paint = model.material("darker_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    liner_steel = model.material("liner_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.22, 0.29, 0.32, 0.42))
    knob_polymer = model.material("knob_polymer", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_polymer = model.material("trim_polymer", rgba=(0.09, 0.10, 0.11, 1.0))
    zinc_fastener = model.material("zinc_fastener", rgba=(0.56, 0.58, 0.61, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.05, 0.05, 0.06, 1.0))
    rack_steel = model.material("rack_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    rubber_foot = model.material("rubber_foot", rgba=(0.12, 0.11, 0.11, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((OVEN_WIDTH, OVEN_DEPTH, BODY_HEIGHT + FOOT_HEIGHT)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + FOOT_HEIGHT) / 2.0)),
    )

    # Outer body shell and seams.
    chassis.visual(
        Box((0.072, OVEN_DEPTH, 0.018)),
        origin=Origin(xyz=(LEFT_X + 0.036, 0.0, FOOT_HEIGHT + 0.009)),
        material=body_paint,
        name="bottom_left_rail",
    )
    chassis.visual(
        Box((0.072, OVEN_DEPTH, 0.018)),
        origin=Origin(xyz=(RIGHT_X - 0.036, 0.0, FOOT_HEIGHT + 0.009)),
        material=body_paint,
        name="bottom_right_rail",
    )
    chassis.visual(
        Box((OVEN_WIDTH - 0.144, 0.082, 0.018)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.041, FOOT_HEIGHT + 0.009)),
        material=body_paint,
        name="bottom_rear_rail",
    )
    chassis.visual(
        Box((0.060, 0.048, 0.018)),
        origin=Origin(xyz=(-0.250, FRONT_Y + 0.052, FOOT_HEIGHT + 0.009)),
        material=body_paint,
        name="bottom_left_front_pad",
    )
    chassis.visual(
        Box((0.086, 0.048, 0.018)),
        origin=Origin(xyz=(0.239, FRONT_Y + 0.052, FOOT_HEIGHT + 0.009)),
        material=body_paint,
        name="bottom_right_front_pad",
    )
    chassis.visual(
        Box((OVEN_WIDTH, OVEN_DEPTH, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + BODY_HEIGHT - 0.009)),
        material=body_paint,
        name="top_shell",
    )
    chassis.visual(
        Box((0.018, OVEN_DEPTH, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(LEFT_X + 0.009, 0.0, 0.189)),
        material=body_paint,
        name="left_shell",
    )
    chassis.visual(
        Box((0.018, OVEN_DEPTH, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(RIGHT_X - 0.009, 0.0, 0.189)),
        material=body_paint,
        name="right_shell",
    )
    chassis.visual(
        Box((OVEN_WIDTH - 0.036, 0.014, BODY_HEIGHT - 0.042)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.007, 0.182)),
        material=body_paint,
        name="rear_service_panel",
    )
    chassis.visual(
        Box((0.028, 0.018, 0.284)),
        origin=Origin(xyz=(-0.266, FRONT_Y + 0.009, 0.184)),
        material=body_paint,
        name="front_left_stile",
    )
    chassis.visual(
        Box((0.016, 0.018, 0.284)),
        origin=Origin(xyz=(0.092, FRONT_Y + 0.009, 0.184)),
        material=body_paint,
        name="front_separator_stile",
    )
    chassis.visual(
        Box((DOOR_WIDTH, 0.018, 0.044)),
        origin=Origin(xyz=(DOOR_CENTER_X, FRONT_Y + 0.009, 0.304)),
        material=body_paint,
        name="front_header",
    )
    chassis.visual(
        Box((DOOR_WIDTH, 0.018, 0.020)),
        origin=Origin(xyz=(DOOR_CENTER_X, FRONT_Y + 0.009, 0.050)),
        material=body_paint,
        name="door_sill"),
    chassis.visual(
        Box((OVEN_WIDTH, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.009, 0.020)),
        material=darker_paint,
        name="lower_kick_plate",
    )
    chassis.visual(
        Box((FASCIA_WIDTH, 0.018, 0.320)),
        origin=Origin(xyz=(FASCIA_CENTER_X, FRONT_Y + 0.009, 0.190)),
        material=darker_paint,
        name="fascia_plate",
    )
    chassis.visual(
        Box((0.028, OVEN_DEPTH - 0.038, 0.022)),
        origin=Origin(xyz=(LEFT_X + 0.020, 0.0, 0.332)),
        material=darker_paint,
        name="left_top_reinforcement",
    )
    chassis.visual(
        Box((0.028, OVEN_DEPTH - 0.038, 0.022)),
        origin=Origin(xyz=(RIGHT_X - 0.020, 0.0, 0.332)),
        material=darker_paint,
        name="right_top_reinforcement",
    )
    chassis.visual(
        Box((OVEN_WIDTH - 0.050, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.020, 0.316)),
        material=darker_paint,
        name="rear_top_stiffener",
    )
    chassis.visual(
        Box((OVEN_WIDTH - 0.040, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.010, 0.336)),
        material=trim_polymer,
        name="top_front_cap",
    )
    chassis.visual(
        Box((0.006, 0.280, 0.182)),
        origin=Origin(xyz=(RIGHT_X - 0.003, 0.0, 0.176)),
        material=trim_polymer,
        name="right_side_service_cover",
    )
    chassis.visual(
        Box((0.006, 0.250, 0.154)),
        origin=Origin(xyz=(LEFT_X + 0.003, 0.0, 0.164)),
        material=trim_polymer,
        name="left_side_guard",
    )

    # Interior cavity liner.
    chassis.visual(
        Box((CAVITY_WIDTH, CAVITY_DEPTH, LINER_THICKNESS)),
        origin=Origin(xyz=(DOOR_CENTER_X, CAVITY_CENTER_Y, CAVITY_FLOOR_Z)),
        material=liner_steel,
        name="cavity_floor",
    )
    chassis.visual(
        Box((CAVITY_WIDTH, CAVITY_DEPTH, LINER_THICKNESS)),
        origin=Origin(xyz=(DOOR_CENTER_X, CAVITY_CENTER_Y, CAVITY_CEILING_Z)),
        material=liner_steel,
        name="cavity_ceiling",
    )
    chassis.visual(
        Box((LINER_THICKNESS, CAVITY_DEPTH, CAVITY_HEIGHT)),
        origin=Origin(
            xyz=(DOOR_CENTER_X - CAVITY_WIDTH / 2.0 - LINER_THICKNESS / 2.0, CAVITY_CENTER_Y, CAVITY_CENTER_Z)
        ),
        material=liner_steel,
        name="cavity_left_wall",
    )
    chassis.visual(
        Box((LINER_THICKNESS, CAVITY_DEPTH, CAVITY_HEIGHT)),
        origin=Origin(
            xyz=(DOOR_CENTER_X + CAVITY_WIDTH / 2.0 + LINER_THICKNESS / 2.0, CAVITY_CENTER_Y, CAVITY_CENTER_Z)
        ),
        material=liner_steel,
        name="cavity_right_wall",
    )
    chassis.visual(
        Box((CAVITY_WIDTH, LINER_THICKNESS, CAVITY_HEIGHT)),
        origin=Origin(xyz=(DOOR_CENTER_X, CAVITY_CENTER_Y + CAVITY_DEPTH / 2.0, CAVITY_CENTER_Z)),
        material=liner_steel,
        name="cavity_rear_wall",
    )

    # Rack and tray guides.
    for side_sign, name_prefix in ((-1.0, "left"), (1.0, "right")):
        rack_guide_x = DOOR_CENTER_X + side_sign * 0.156
        chassis.visual(
            Box((0.016, RACK_DEPTH, 0.008)),
            origin=Origin(xyz=(rack_guide_x, RACK_GUIDE_Y, RACK_GUIDE_Z)),
            material=liner_steel,
            name=f"{name_prefix}_rack_guide",
        )

    # Hinge brackets and fascia bezels.
    for x_value, bracket_name in (
        (-0.251, "left_hinge_bracket"),
        (0.079, "right_hinge_bracket"),
    ):
        chassis.visual(
            Box((0.014, 0.018, 0.022)),
            origin=Origin(xyz=(x_value, FRONT_Y + 0.014, DOOR_HINGE_Z + 0.001)),
            material=darker_paint,
            name=bracket_name,
        )

    knob_x = FASCIA_CENTER_X
    for index, knob_z in enumerate((0.276, 0.200, 0.124), start=1):
        chassis.visual(
            Cylinder(radius=0.030, length=0.004),
            origin=Origin(
                xyz=(knob_x, FRONT_Y - 0.001, knob_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_polymer,
            name=f"knob_bezel_{index}",
        )
    # Feet.
    for index, (x_value, y_value) in enumerate(
        (
            (-0.215, -0.150),
            (0.215, -0.150),
            (-0.215, 0.150),
            (0.215, 0.150),
        ),
        start=1,
    ):
        chassis.visual(
            Cylinder(radius=0.025, length=FOOT_HEIGHT),
            origin=Origin(xyz=(x_value, y_value, FOOT_HEIGHT / 2.0)),
            material=rubber_foot,
            name=f"foot_{index}",
        )

    # Exposed service fasteners.
    for index, x_value in enumerate((-0.245, 0.066, 0.150, 0.234), start=1):
        _add_front_screw(
            chassis,
            x=x_value,
            y=FRONT_Y + 0.002,
            z=0.328,
            radius=0.005,
            depth=0.006,
            material=zinc_fastener,
            name=f"front_top_screw_{index}",
        )
    for index, z_value in enumerate((0.286, 0.206, 0.126, 0.046), start=1):
        _add_side_screw(
            chassis,
            x=RIGHT_X - 0.001,
            y=0.095,
            z=z_value,
            radius=0.0045,
            depth=0.006,
            material=zinc_fastener,
            name=f"right_service_screw_{index}",
        )
        _add_side_screw(
            chassis,
            x=LEFT_X + 0.001,
            y=0.065,
            z=z_value + 0.012,
            radius=0.0045,
            depth=0.006,
            material=zinc_fastener,
            name=f"left_service_screw_{index}",
        )
    for index, x_value in enumerate((-0.190, -0.070, 0.050, 0.170), start=1):
        _add_front_screw(
            chassis,
            x=x_value,
            y=REAR_Y - 0.013,
            z=0.182,
            radius=0.0045,
            depth=0.006,
            material=zinc_fastener,
            name=f"rear_panel_screw_{index}",
        )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.006, DOOR_HEIGHT / 2.0)),
    )
    door.visual(
        Box((DOOR_WIDTH - 0.080, DOOR_THICKNESS, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=darker_paint,
        name="door_lower_frame",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=darker_paint,
        name="door_upper_frame",
    )
    door.visual(
        Box((0.022, DOOR_THICKNESS, 0.170)),
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0 + 0.011, 0.0, 0.163)),
        material=darker_paint,
        name="door_left_frame",
    )
    door.visual(
        Box((0.022, DOOR_THICKNESS, 0.170)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 - 0.011, 0.0, 0.163)),
        material=darker_paint,
        name="door_right_frame",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.092, DOOR_THICKNESS, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=trim_polymer,
        name="door_kick_plate",
    )
    door.visual(
        Box((0.286, 0.010, 0.156)),
        origin=Origin(xyz=(0.0, 0.004, 0.146)),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((0.306, 0.004, 0.176)),
        origin=Origin(xyz=(0.0, 0.009, 0.146)),
        material=gasket_black,
        name="door_glass_gasket",
    )
    door.visual(
        Box((0.240, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.038, 0.205)),
        material=trim_polymer,
        name="door_handle",
    )
    door.visual(
        Box((0.022, 0.040, 0.024)),
        origin=Origin(xyz=(-0.095, -0.020, 0.205)),
        material=trim_polymer,
        name="door_handle_left_bracket",
    )
    door.visual(
        Box((0.022, 0.040, 0.024)),
        origin=Origin(xyz=(0.095, -0.020, 0.205)),
        material=trim_polymer,
        name="door_handle_right_bracket",
    )
    door.visual(
        Box((0.012, 0.016, 0.070)),
        origin=Origin(xyz=(-0.165, 0.0, 0.047)),
        material=zinc_fastener,
        name="door_left_hinge_ear",
    )
    door.visual(
        Box((0.012, 0.016, 0.070)),
        origin=Origin(xyz=(0.165, 0.0, 0.047)),
        material=zinc_fastener,
        name="door_right_hinge_ear",
    )
    for index, (x_value, screw_z) in enumerate(
        ((-0.128, 0.214), (0.128, 0.214), (-0.108, 0.036), (0.108, 0.036)),
        start=1,
    ):
        _add_front_screw(
            door,
            x=x_value,
            y=-0.006,
            z=screw_z,
            radius=0.004,
            depth=0.005,
            material=zinc_fastener,
            name=f"door_frame_screw_{index}",
        )

    function_knob = model.part("function_knob")
    function_knob.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_polymer,
        name="function_knob_body",
    )
    function_knob.visual(
        Cylinder(radius=0.027, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_polymer,
        name="function_knob_collar",
    )
    function_knob.visual(
        Box((0.006, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.023, 0.015)),
        material=zinc_fastener,
        name="function_knob_indicator",
    )
    function_knob.inertial = Inertial.from_geometry(
        Box((0.056, 0.030, 0.056)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
    )

    temp_knob = model.part("temp_knob")
    temp_knob.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_polymer,
        name="temp_knob_body",
    )
    temp_knob.visual(
        Cylinder(radius=0.029, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_polymer,
        name="temp_knob_collar",
    )
    temp_knob.visual(
        Box((0.006, 0.004, 0.015)),
        origin=Origin(xyz=(0.0, -0.023, 0.017)),
        material=zinc_fastener,
        name="temp_knob_indicator",
    )
    temp_knob.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.060)),
        mass=0.085,
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_polymer,
        name="timer_knob_body",
    )
    timer_knob.visual(
        Cylinder(radius=0.027, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_polymer,
        name="timer_knob_collar",
    )
    timer_knob.visual(
        Box((0.006, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.023, 0.015)),
        material=zinc_fastener,
        name="timer_knob_indicator",
    )
    timer_knob.inertial = Inertial.from_geometry(
        Box((0.056, 0.030, 0.056)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
    )

    rack = model.part("rack")
    rack.inertial = Inertial.from_geometry(
        Box((RACK_WIDTH, RACK_DEPTH, 0.028)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    for side_sign, name_prefix in ((-1.0, "left"), (1.0, "right")):
        rack.visual(
            Box((0.012, 0.288, 0.008)),
            origin=Origin(xyz=(side_sign * 0.156, 0.0, 0.0)),
            material=rack_steel,
            name=f"{name_prefix}_rack_runner",
        )
        rack.visual(
            Box((0.010, 0.226, 0.010)),
            origin=Origin(xyz=(side_sign * 0.151, 0.0, 0.008)),
            material=rack_steel,
            name=f"{name_prefix}_rack_side_bar",
        )
    rack.visual(
        Box((RACK_WIDTH - 0.020, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.145, 0.008)),
        material=rack_steel,
        name="rack_front_bar",
    )
    rack.visual(
        Box((RACK_WIDTH - 0.020, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.136, 0.008)),
        material=rack_steel,
        name="rack_rear_bar",
    )
    for index, y_value in enumerate((-0.095, -0.045, 0.005, 0.055, 0.105), start=1):
        rack.visual(
            Box((RACK_WIDTH - 0.018, 0.008, 0.006)),
            origin=Origin(xyz=(0.0, y_value, 0.006)),
            material=rack_steel,
            name=f"rack_cross_bar_{index}",
        )
    rack.visual(
        Box((0.120, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -0.158, 0.018)),
        material=trim_polymer,
        name="rack_handle",
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.inertial = Inertial.from_geometry(
        Box((TRAY_WIDTH + 0.018, TRAY_DEPTH, 0.028)),
        mass=0.32,
        origin=Origin(xyz=(0.0, -0.004, 0.010)),
    )
    crumb_tray.visual(
        Box((0.236, TRAY_DEPTH - 0.034, 0.004)),
        origin=Origin(xyz=(0.0, -0.006, -0.010)),
        material=liner_steel,
        name="tray_floor",
    )
    crumb_tray.visual(
        Box((0.008, 0.164, 0.008)),
        origin=Origin(xyz=(-0.118, 0.036, -0.006)),
        material=liner_steel,
        name="tray_left_flange",
    )
    crumb_tray.visual(
        Box((0.008, 0.164, 0.008)),
        origin=Origin(xyz=(0.118, 0.036, -0.006)),
        material=liner_steel,
        name="tray_right_flange",
    )
    crumb_tray.visual(
        Box((0.236, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.110, -0.008)),
        material=liner_steel,
        name="tray_rear_lip",
    )
    crumb_tray.visual(
        Box((0.236, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, -0.122, -0.004)),
        material=liner_steel,
        name="tray_front_wall",
    )
    crumb_tray.visual(
        Box((0.248, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.146, -0.006)),
        material=trim_polymer,
        name="tray_handle",
    )
    crumb_tray.visual(
        Box((0.020, 0.026, 0.010)),
        origin=Origin(xyz=(-0.086, -0.142, -0.004)),
        material=liner_steel,
        name="tray_handle_left_bracket",
    )
    crumb_tray.visual(
        Box((0.020, 0.026, 0.010)),
        origin=Origin(xyz=(0.086, -0.142, -0.004)),
        material=liner_steel,
        name="tray_handle_right_bracket",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "function_knob_turn",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=function_knob,
        origin=Origin(xyz=(knob_x, FRONT_Y, 0.276)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=6.0,
            lower=0.0,
            upper=5.40,
        ),
    )
    model.articulation(
        "temp_knob_turn",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=temp_knob,
        origin=Origin(xyz=(knob_x, FRONT_Y, 0.200)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=6.0,
            lower=0.0,
            upper=5.40,
        ),
    )
    model.articulation(
        "timer_knob_turn",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=timer_knob,
        origin=Origin(xyz=(knob_x, FRONT_Y, 0.124)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=6.0,
            lower=0.0,
            upper=5.40,
        ),
    )
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=rack,
        origin=Origin(xyz=(DOOR_CENTER_X, -0.052, RACK_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=9.0,
            velocity=0.20,
            lower=0.0,
            upper=0.160,
        ),
    )
    model.articulation(
        "crumb_tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=crumb_tray,
        origin=Origin(xyz=(DOOR_CENTER_X, -0.030, TRAY_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.110,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    door = object_model.get_part("door")
    function_knob = object_model.get_part("function_knob")
    temp_knob = object_model.get_part("temp_knob")
    timer_knob = object_model.get_part("timer_knob")
    rack = object_model.get_part("rack")
    crumb_tray = object_model.get_part("crumb_tray")

    door_hinge = object_model.get_articulation("door_hinge")
    function_knob_turn = object_model.get_articulation("function_knob_turn")
    temp_knob_turn = object_model.get_articulation("temp_knob_turn")
    timer_knob_turn = object_model.get_articulation("timer_knob_turn")
    rack_slide = object_model.get_articulation("rack_slide")
    crumb_tray_slide = object_model.get_articulation("crumb_tray_slide")

    door_glass = door.get_visual("door_glass")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(door, chassis, name="door_connected_to_chassis_at_rest")
    ctx.expect_contact(function_knob, chassis, name="function_knob_mounted")
    ctx.expect_contact(temp_knob, chassis, name="temp_knob_mounted")
    ctx.expect_contact(timer_knob, chassis, name="timer_knob_mounted")
    ctx.expect_contact(rack, chassis, name="rack_supported")
    ctx.expect_contact(crumb_tray, chassis, name="crumb_tray_supported")

    ctx.expect_overlap(door, chassis, axes="xz", min_overlap=0.20, name="door_covers_opening_zone")
    rack_aabb = ctx.part_world_aabb(rack)
    tray_aabb = ctx.part_world_aabb(crumb_tray)
    cavity_floor_aabb = ctx.part_element_world_aabb(chassis, elem="cavity_floor")
    cavity_ceiling_aabb = ctx.part_element_world_aabb(chassis, elem="cavity_ceiling")
    cavity_rear_wall_aabb = ctx.part_element_world_aabb(chassis, elem="cavity_rear_wall")
    assert rack_aabb is not None
    assert tray_aabb is not None
    assert cavity_floor_aabb is not None
    assert cavity_ceiling_aabb is not None
    assert cavity_rear_wall_aabb is not None
    ctx.check(
        "rack_aligned_with_cavity",
        rack_aabb[0][0] >= cavity_floor_aabb[0][0] - 0.001
        and rack_aabb[1][0] <= cavity_floor_aabb[1][0] + 0.001
        and rack_aabb[0][2] >= cavity_floor_aabb[1][2] - 0.002
        and rack_aabb[1][2] <= cavity_ceiling_aabb[0][2] + 0.002
        and rack_aabb[1][1] <= cavity_rear_wall_aabb[0][1] + 0.002,
        details=f"rack={rack_aabb}, cavity_floor={cavity_floor_aabb}, cavity_ceiling={cavity_ceiling_aabb}, rear_wall={cavity_rear_wall_aabb}",
    )
    ctx.check(
        "crumb_tray_aligned_with_slot",
        tray_aabb[0][0] >= cavity_floor_aabb[0][0] - 0.004
        and tray_aabb[1][0] <= cavity_floor_aabb[1][0] + 0.004
        and tray_aabb[1][2] <= cavity_floor_aabb[0][2] + 0.004
        and tray_aabb[0][2] >= 0.0,
        details=f"tray={tray_aabb}, cavity_floor={cavity_floor_aabb}",
    )
    ctx.expect_within(rack, chassis, axes="x", margin=0.0, name="rack_within_body_width")
    ctx.expect_within(crumb_tray, chassis, axes="x", margin=0.0, name="crumb_tray_within_body_width")
    ctx.expect_within(door, door, axes="xz", inner_elem="door_glass", margin=0.0, name="door_glass_within_frame")
    ctx.expect_overlap(function_knob, chassis, axes="xz", min_overlap=0.03, elem_b="fascia_plate", name="function_knob_on_fascia")
    ctx.expect_overlap(temp_knob, chassis, axes="xz", min_overlap=0.03, elem_b="fascia_plate", name="temp_knob_on_fascia")
    ctx.expect_overlap(timer_knob, chassis, axes="xz", min_overlap=0.03, elem_b="fascia_plate", name="timer_knob_on_fascia")

    ctx.check(
        "door_hinge_axis_is_widthwise",
        tuple(round(value, 4) for value in door_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"door_hinge.axis={door_hinge.axis}",
    )
    ctx.check(
        "door_hinge_limit_is_realistic",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and 1.55 <= door_hinge.motion_limits.upper <= 1.80,
        details=f"door_hinge.limits={door_hinge.motion_limits}",
    )
    for name, knob_joint in (
        ("function", function_knob_turn),
        ("temp", temp_knob_turn),
        ("timer", timer_knob_turn),
    ):
        ctx.check(
            f"{name}_knob_axis_is_front_to_back",
            tuple(round(value, 4) for value in knob_joint.axis) == (0.0, 1.0, 0.0),
            details=f"{name}_axis={knob_joint.axis}",
        )
        ctx.check(
            f"{name}_knob_range_is_selector_like",
            knob_joint.motion_limits is not None
            and knob_joint.motion_limits.lower == 0.0
            and knob_joint.motion_limits.upper is not None
            and 5.0 <= knob_joint.motion_limits.upper <= 5.8,
            details=f"{name}_limits={knob_joint.motion_limits}",
        )
    for name, slide_joint, expected_upper in (
        ("rack", rack_slide, 0.160),
        ("crumb_tray", crumb_tray_slide, 0.110),
    ):
        ctx.check(
            f"{name}_slide_axis_is_forward",
            tuple(round(value, 4) for value in slide_joint.axis) == (0.0, -1.0, 0.0),
            details=f"{name}_axis={slide_joint.axis}",
        )
        ctx.check(
            f"{name}_travel_is_bounded",
            slide_joint.motion_limits is not None
            and slide_joint.motion_limits.lower == 0.0
            and slide_joint.motion_limits.upper is not None
            and abs(slide_joint.motion_limits.upper - expected_upper) < 1e-6,
            details=f"{name}_limits={slide_joint.motion_limits}",
        )

    door_rest_aabb = ctx.part_world_aabb(door)
    assert door_rest_aabb is not None
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.check(
            "door_swings_downward",
            door_open_aabb[1][2] < door_rest_aabb[1][2] - 0.20 and door_open_aabb[0][1] < door_rest_aabb[0][1] - 0.12,
            details=f"rest={door_rest_aabb}, open={door_open_aabb}",
        )

    with ctx.pose(
        {
            function_knob_turn: 2.8,
            temp_knob_turn: 3.2,
            timer_knob_turn: 4.0,
        }
    ):
        ctx.expect_contact(function_knob, chassis, name="function_knob_contact_when_rotated")
        ctx.expect_contact(temp_knob, chassis, name="temp_knob_contact_when_rotated")
        ctx.expect_contact(timer_knob, chassis, name="timer_knob_contact_when_rotated")
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")

    rack_rest_aabb = ctx.part_world_aabb(rack)
    assert rack_rest_aabb is not None
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper, rack_slide: rack_slide.motion_limits.upper}):
        rack_open_aabb = ctx.part_world_aabb(rack)
        assert rack_open_aabb is not None
        ctx.expect_contact(rack, chassis, name="rack_supported_when_extended")
        ctx.expect_overlap(rack, door, axes="x", min_overlap=0.20, name="rack_stays_centered_with_door")
        ctx.fail_if_parts_overlap_in_current_pose(name="rack_extended_no_overlap")
        ctx.fail_if_isolated_parts(name="rack_extended_no_floating")
        ctx.check(
            "rack_moves_forward_when_extended",
            rack_open_aabb[0][1] < rack_rest_aabb[0][1] - 0.12,
            details=f"rest={rack_rest_aabb}, open={rack_open_aabb}",
        )

    tray_rest_aabb = ctx.part_world_aabb(crumb_tray)
    assert tray_rest_aabb is not None
    with ctx.pose({crumb_tray_slide: crumb_tray_slide.motion_limits.upper}):
        tray_open_aabb = ctx.part_world_aabb(crumb_tray)
        assert tray_open_aabb is not None
        ctx.expect_contact(crumb_tray, chassis, name="crumb_tray_supported_when_extended")
        ctx.expect_overlap(crumb_tray, chassis, axes="x", min_overlap=0.22, name="crumb_tray_handle_keeps_width_alignment")
        ctx.fail_if_parts_overlap_in_current_pose(name="crumb_tray_extended_no_overlap")
        ctx.fail_if_isolated_parts(name="crumb_tray_extended_no_floating")
        ctx.check(
            "crumb_tray_moves_forward_when_extended",
            tray_open_aabb[0][1] < tray_rest_aabb[0][1] - 0.08,
            details=f"rest={tray_rest_aabb}, open={tray_open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
