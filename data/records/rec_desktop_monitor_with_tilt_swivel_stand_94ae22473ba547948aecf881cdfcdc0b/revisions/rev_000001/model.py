from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _curved_panel_geometry(
    width: float,
    height: float,
    thickness: float,
    curve_depth: float,
    *,
    front_center_y: float,
    segments: int = 40,
) -> MeshGeometry:
    """A horizontally curved rectangular slab, concave toward -Y."""

    geom = MeshGeometry()
    front_bottom = []
    front_top = []
    rear_bottom = []
    rear_top = []

    for i in range(segments + 1):
        t = i / segments
        x = -width * 0.5 + width * t
        sag = curve_depth * (x / (width * 0.5)) ** 2
        y_front = front_center_y - sag
        y_rear = y_front + thickness
        front_bottom.append(geom.add_vertex(x, y_front, -height * 0.5))
        front_top.append(geom.add_vertex(x, y_front, height * 0.5))
        rear_bottom.append(geom.add_vertex(x, y_rear, -height * 0.5))
        rear_top.append(geom.add_vertex(x, y_rear, height * 0.5))

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        # Front and rear curved faces.
        quad(front_bottom[i], front_bottom[i + 1], front_top[i + 1], front_top[i])
        quad(rear_bottom[i + 1], rear_bottom[i], rear_top[i], rear_top[i + 1])
        # Top and bottom lips, so the panel reads as a real shell thickness.
        quad(front_top[i], front_top[i + 1], rear_top[i + 1], rear_top[i])
        quad(front_bottom[i + 1], front_bottom[i], rear_bottom[i], rear_bottom[i + 1])

    # End caps.
    quad(front_bottom[0], front_top[0], rear_top[0], rear_bottom[0])
    quad(front_bottom[-1], rear_bottom[-1], rear_top[-1], front_top[-1])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_gaming_monitor")

    matte_black = model.material("matte_black", rgba=(0.006, 0.007, 0.009, 1.0))
    graphite = model.material("graphite_shell", rgba=(0.08, 0.085, 0.095, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.012, 0.018, 0.035, 1.0))
    red = model.material("red_power_mark", rgba=(0.75, 0.04, 0.03, 1.0))

    base = model.part("base")
    # Low, wide three-foot gaming-monitor stand.
    base.visual(
        Cylinder(radius=0.072, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_metal,
        name="center_hub",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_metal,
        name="swivel_socket",
    )
    for name, yaw, length in (
        ("front_foot", -math.pi / 2.0, 0.42),
        ("rear_foot_0", math.radians(35.0), 0.34),
        ("rear_foot_1", math.radians(145.0), 0.34),
    ):
        # Each rectangular foot overlaps the round hub at its inner end, making
        # the root part a single connected tripod base.
        cx = math.cos(yaw) * length * 0.34
        cy = math.sin(yaw) * length * 0.34
        base.visual(
            Box((length, 0.070, 0.022)),
            origin=Origin(xyz=(cx, cy, 0.011), rpy=(0.0, 0.0, yaw)),
            material=dark_metal,
            name=name,
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.027, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=dark_metal,
        name="upright_post",
    )
    yoke.visual(
        Box((0.155, 0.068, 0.074)),
        origin=Origin(xyz=(0.0, 0.055, 0.330)),
        material=dark_metal,
        name="split_hub",
    )
    for side, sx in (("0", -1.0), ("1", 1.0)):
        yoke.visual(
            Box((0.440, 0.046, 0.035)),
            origin=Origin(xyz=(sx * 0.285, 0.095, 0.360)),
            material=dark_metal,
            name=f"yoke_arm_{side}",
        )
        yoke.visual(
            Box((0.042, 0.085, 0.118)),
            origin=Origin(xyz=(sx * 0.514, 0.055, 0.360)),
            material=dark_metal,
            name=f"end_cheek_{side}",
        )
        yoke.visual(
            Cylinder(radius=0.032, length=0.030),
            origin=Origin(
                xyz=(sx * 0.533, 0.055, 0.360),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name=f"bearing_{side}",
        )

    display = model.part("display")
    shell_mesh = mesh_from_geometry(
        _curved_panel_geometry(
            0.960,
            0.430,
            0.055,
            0.055,
            front_center_y=-0.115,
            segments=48,
        ),
        "curved_display_shell",
    )
    display.visual(shell_mesh, material=graphite, name="curved_shell")
    screen_mesh = mesh_from_geometry(
        _curved_panel_geometry(
            0.880,
            0.340,
            0.004,
            0.055,
            front_center_y=-0.181,
            segments=48,
        ),
        "curved_screen_glass",
    )
    display.visual(screen_mesh, material=glass, name="screen")
    # Bottom-heavy bezel lip, visible below the glass and carrying the controls.
    display.visual(
        Box((0.900, 0.120, 0.025)),
        origin=Origin(xyz=(0.0, -0.155, -0.204)),
        material=matte_black,
        name="bottom_bezel_lip",
    )
    display.visual(
        Box((0.900, 0.120, 0.026)),
        origin=Origin(xyz=(0.0, -0.155, 0.181)),
        material=matte_black,
        name="top_bezel_lip",
    )
    display.visual(
        Box((0.155, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.125, 0.198)),
        material=matte_black,
        name="top_brand_tab",
    )
    for side, sx in (("0", -1.0), ("1", 1.0)):
        # Rear bosses bridge the curved shell forward of the tilt axis to the
        # exposed side trunnions; the yoke arms carry these pins visibly.
        display.visual(
            Box((0.105, 0.124, 0.095)),
            origin=Origin(xyz=(sx * 0.435, -0.058, 0.0)),
            material=graphite,
            name=f"rear_boss_{side}",
        )
        display.visual(
            Cylinder(radius=0.026, length=0.036),
            origin=Origin(
                xyz=(sx * 0.500, 0.020, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name=f"trunnion_{side}",
        )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Cylinder(radius=0.004, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="pivot_pin",
    )
    power_rocker.visual(
        Box((0.044, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=rubber,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((0.012, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, -0.0125, -0.017)),
        material=red,
        name="power_mark",
    )

    button_xs = (-0.064, -0.026, 0.012)
    for i, _ in enumerate(button_xs):
        button = model.part(f"menu_button_{i}")
        button.visual(
            Box((0.026, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=rubber,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=matte_black,
            name="lower_step",
        )

    swivel = model.articulation(
        "base_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5),
    )
    model.articulation(
        "yoke_to_display",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=display,
        origin=Origin(xyz=(0.0, 0.020, 0.360)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.25, upper=0.35),
    )
    model.articulation(
        "display_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=display,
        child=power_rocker,
        origin=Origin(xyz=(0.070, -0.176, -0.2165)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-0.22, upper=0.22),
    )
    for i, x in enumerate(button_xs):
        model.articulation(
            f"display_to_menu_button_{i}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=model.get_part(f"menu_button_{i}"),
            origin=Origin(xyz=(x, -0.176, -0.2165)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    yoke = object_model.get_part("yoke")
    display = object_model.get_part("display")

    ctx.expect_contact(
        yoke,
        display,
        elem_a="bearing_0",
        elem_b="trunnion_0",
        contact_tol=0.002,
        name="left yoke bearing visibly supports display trunnion",
    )
    ctx.expect_contact(
        yoke,
        display,
        elem_a="bearing_1",
        elem_b="trunnion_1",
        contact_tol=0.002,
        name="right yoke bearing visibly supports display trunnion",
    )
    for side in ("0", "1"):
        ctx.allow_overlap(
            display,
            yoke,
            elem_a=f"trunnion_{side}",
            elem_b=f"end_cheek_{side}",
            reason="The display trunnion is intentionally captured through a simplified solid yoke cheek with an implied bore.",
        )
        ctx.expect_overlap(
            display,
            yoke,
            axes="yz",
            elem_a=f"trunnion_{side}",
            elem_b=f"end_cheek_{side}",
            min_overlap=0.020,
            name=f"trunnion_{side} remains captured in the yoke cheek",
        )
    ctx.expect_gap(
        "menu_button_1",
        "menu_button_0",
        axis="x",
        min_gap=0.006,
        name="menu buttons remain separate caps",
    )
    ctx.expect_gap(
        "power_rocker",
        "menu_button_2",
        axis="x",
        min_gap=0.018,
        name="power rocker is distinct from menu buttons",
    )

    tilt = object_model.get_articulation("yoke_to_display")
    screen_closed = ctx.part_element_world_aabb(display, elem="screen")
    with ctx.pose({tilt: 0.30}):
        screen_tilted = ctx.part_element_world_aabb(display, elem="screen")
    ctx.check(
        "display tilts on horizontal yoke axis",
        screen_closed is not None
        and screen_tilted is not None
        and abs(screen_tilted[1][1] - screen_closed[1][1]) > 0.020,
        details=f"closed={screen_closed}, tilted={screen_tilted}",
    )

    swivel = object_model.get_articulation("base_to_yoke")
    ctx.check(
        "stand swivel is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={swivel.articulation_type}",
    )

    b0 = object_model.get_part("menu_button_0")
    b1 = object_model.get_part("menu_button_1")
    b0_joint = object_model.get_articulation("display_to_menu_button_0")
    rest_b0 = ctx.part_world_position(b0)
    rest_b1 = ctx.part_world_position(b1)
    with ctx.pose({b0_joint: 0.004}):
        pressed_b0 = ctx.part_world_position(b0)
        still_b1 = ctx.part_world_position(b1)
    ctx.check(
        "menu button depresses independently",
        rest_b0 is not None
        and pressed_b0 is not None
        and rest_b1 is not None
        and still_b1 is not None
        and pressed_b0[2] > rest_b0[2] + 0.003
        and abs(still_b1[2] - rest_b1[2]) < 0.0005,
        details=f"b0={rest_b0}->{pressed_b0}, b1={rest_b1}->{still_b1}",
    )

    return ctx.report()


object_model = build_object_model()
