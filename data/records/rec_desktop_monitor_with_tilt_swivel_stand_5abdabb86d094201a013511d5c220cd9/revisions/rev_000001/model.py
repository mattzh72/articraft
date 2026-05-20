from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


DISPLAY_OUTER = (0.560, 0.355)
DISPLAY_OPENING = (0.512, 0.285)
DISPLAY_CENTER_Z = 0.020
DISPLAY_FRONT_Y = -0.052
DISPLAY_BACK_Y = -0.008
BASE_HEIGHT = 0.018
SWIVEL_Z = 0.028
NECK_HEIGHT = 0.308


def _make_button(
    model: ArticulatedObject,
    display,
    name: str,
    x: float,
    z: float,
    *,
    round_button: bool = False,
    label_width: float = 0.007,
) -> None:
    """Create one independently pressed monitor menu button."""

    button = model.part(name)
    if round_button:
        button.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
            material="button_black",
            name="cap",
        )
        button.visual(
            Box((0.0018, 0.0007, 0.0065)),
            origin=Origin(xyz=(0.0, -0.0062, 0.0)),
            material="control_icon",
            name="power_mark",
        )
    else:
        button.visual(
            Box((0.018, 0.006, 0.008)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material="button_black",
            name="cap",
        )
        button.visual(
            Box((label_width, 0.0008, 0.0012)),
            origin=Origin(xyz=(0.0, -0.0062, 0.0)),
            material="control_icon",
            name="front_mark",
        )

    model.articulation(
        f"display_to_{name}",
        ArticulationType.PRISMATIC,
        parent=display,
        child=button,
        origin=Origin(xyz=(x, DISPLAY_FRONT_Y - 0.001, z)),
        # Buttons are proud of the front face and press inward toward +Y.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.04, lower=0.0, upper=0.004),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_monitor_24in")
    model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    model.material("soft_black", rgba=(0.045, 0.047, 0.050, 1.0))
    model.material("screen_glass", rgba=(0.010, 0.014, 0.020, 1.0))
    model.material("button_black", rgba=(0.020, 0.022, 0.024, 1.0))
    model.material("control_icon", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    model.material("bearing_ring", rgba=(0.18, 0.18, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.280, 0.205, 0.034, corner_segments=10),
                BASE_HEIGHT,
                center=True,
            ),
            "rounded_pedestal",
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material="matte_black",
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.002)),
        material="bearing_ring",
        name="swivel_ring",
    )
    base.visual(
        Cylinder(radius=0.044, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.006)),
        material="soft_black",
        name="bearing_puck",
    )
    for i, (x, y) in enumerate(((-0.095, -0.070), (0.095, -0.070), (-0.095, 0.070), (0.095, 0.070))):
        base.visual(
            Box((0.050, 0.020, 0.003)),
            origin=Origin(xyz=(x, y, 0.0015)),
            material="rubber",
            name=f"rubber_foot_{i}",
        )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="soft_black",
        name="turntable",
    )
    neck.visual(
        Box((0.045, 0.032, 0.250)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material="matte_black",
        name="central_neck",
    )
    neck.visual(
        Box((0.064, 0.037, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="soft_black",
        name="lower_collar",
    )
    neck.visual(
        Box((0.100, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.268)),
        material="soft_black",
        name="top_bridge",
    )
    neck.visual(
        Box((0.014, 0.044, 0.080)),
        origin=Origin(xyz=(-0.043, 0.0, NECK_HEIGHT)),
        material="matte_black",
        name="hinge_ear_0",
    )
    neck.visual(
        Box((0.014, 0.044, 0.080)),
        origin=Origin(xyz=(0.043, 0.0, NECK_HEIGHT)),
        material="matte_black",
        name="hinge_ear_1",
    )
    neck.visual(
        Cylinder(radius=0.008, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, NECK_HEIGHT), rpy=(0.0, pi / 2, 0.0)),
        material="bearing_ring",
        name="hinge_pin",
    )

    model.articulation(
        "base_to_neck",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0),
    )

    display = model.part("display")
    display.visual(
        Box((DISPLAY_OUTER[0], 0.026, DISPLAY_OUTER[1])),
        origin=Origin(xyz=(0.0, -0.035, DISPLAY_CENTER_Z)),
        material="soft_black",
        name="rear_shell",
    )
    display.visual(
        mesh_from_geometry(
            BezelGeometry(
                DISPLAY_OPENING,
                DISPLAY_OUTER,
                0.012,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.004,
                outer_corner_radius=0.014,
                wall=(0.024, 0.024, 0.020, 0.050),
            ),
            "display_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.046, DISPLAY_CENTER_Z), rpy=(pi / 2, 0.0, 0.0)),
        material="matte_black",
        name="front_bezel",
    )
    display.visual(
        Box((0.522, 0.002, 0.292)),
        origin=Origin(xyz=(0.0, -0.053, DISPLAY_CENTER_Z + 0.015)),
        material="screen_glass",
        name="screen_glass",
    )
    display.visual(
        Box((0.150, 0.009, 0.020)),
        origin=Origin(xyz=(0.185, -0.0485, DISPLAY_CENTER_Z - 0.147)),
        material="soft_black",
        name="control_recess",
    )
    display.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material="soft_black",
        name="hinge_socket",
    )
    display.visual(
        Box((0.072, 0.016, 0.052)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material="soft_black",
        name="hinge_mount",
    )

    model.articulation(
        "neck_to_display",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, NECK_HEIGHT)),
        # Positive values tilt the top of the screen backward, like a desk monitor.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.12, upper=0.35),
    )

    button_z = DISPLAY_CENTER_Z - 0.147
    for name, x, width in (
        ("menu_button", 0.128, 0.010),
        ("minus_button", 0.154, 0.008),
        ("plus_button", 0.180, 0.008),
        ("select_button", 0.206, 0.010),
    ):
        _make_button(model, display, name, x, button_z, label_width=width)
    _make_button(model, display, "power_button", 0.235, button_z, round_button=True)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    neck = object_model.get_part("neck")
    display = object_model.get_part("display")
    tilt = object_model.get_articulation("neck_to_display")
    swivel = object_model.get_articulation("base_to_neck")

    ctx.allow_overlap(
        neck,
        display,
        elem_a="hinge_pin",
        elem_b="hinge_socket",
        reason="The metal hinge pin is intentionally captured inside the display tilt socket.",
    )
    ctx.expect_overlap(
        neck,
        display,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_socket",
        min_overlap=0.050,
        name="tilt hinge pin captured across the socket",
    )

    display_aabb = ctx.part_world_aabb(display)
    ctx.check("24 inch class display width", display_aabb is not None and 0.54 <= display_aabb[1][0] - display_aabb[0][0] <= 0.59, details=f"aabb={display_aabb}")
    ctx.check("office monitor display height", display_aabb is not None and 0.34 <= display_aabb[1][2] - display_aabb[0][2] <= 0.38, details=f"aabb={display_aabb}")

    ctx.expect_contact(
        base,
        neck,
        elem_a="bearing_puck",
        elem_b="turntable",
        contact_tol=0.001,
        name="swivel turntable seated on base bearing",
    )
    ctx.check("swivel joint is continuous", swivel.articulation_type == ArticulationType.CONTINUOUS, details=str(swivel.articulation_type))
    ctx.check("tilt hinge is horizontal", abs(tilt.axis[0]) > 0.99 and abs(tilt.axis[1]) < 0.01 and abs(tilt.axis[2]) < 0.01, details=f"axis={tilt.axis}")

    rest_tilt_aabb = ctx.part_world_aabb(display)
    with ctx.pose({tilt: 0.30}):
        tilted_aabb = ctx.part_world_aabb(display)
    ctx.check(
        "positive tilt moves screen rearward",
        rest_tilt_aabb is not None and tilted_aabb is not None and tilted_aabb[1][1] > rest_tilt_aabb[1][1] + 0.010,
        details=f"rest={rest_tilt_aabb}, tilted={tilted_aabb}",
    )

    button_names = ("menu_button", "minus_button", "plus_button", "select_button", "power_button")
    ctx.check("five separate front buttons", all(object_model.get_part(name) is not None for name in button_names), details=str(button_names))
    for name in button_names:
        button = object_model.get_part(name)
        joint = object_model.get_articulation(f"display_to_{name}")
        ctx.expect_contact(
            display,
            button,
            elem_a="control_recess",
            elem_b="cap",
            contact_tol=0.002,
            name=f"{name} cap seated in lower bezel",
        )
        before = ctx.part_world_position(button)
        with ctx.pose({joint: 0.004}):
            after = ctx.part_world_position(button)
        ctx.check(
            f"{name} presses inward independently",
            before is not None and after is not None and after[1] > before[1] + 0.003,
            details=f"before={before}, after={after}",
        )

    return ctx.report()


object_model = build_object_model()
