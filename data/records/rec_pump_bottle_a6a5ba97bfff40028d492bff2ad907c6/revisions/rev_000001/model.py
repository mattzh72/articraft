from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _build_bottle_shell():
    outer_profile = [
        (0.000, 0.000),
        (0.040, 0.004),
        (0.045, 0.012),
        (0.047, 0.070),
        (0.046, 0.102),
        (0.042, 0.118),
        (0.031, 0.132),
        (0.018, 0.143),
        (0.013, 0.156),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.036, 0.008),
        (0.039, 0.014),
        (0.040, 0.070),
        (0.039, 0.099),
        (0.035, 0.113),
        (0.025, 0.126),
        (0.0145, 0.137),
        (0.0105, 0.154),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "dish_soap_bottle_shell",
    )


def _build_collar_shell():
    outer_profile = [
        (0.0215, 0.000),
        (0.0215, 0.004),
        (0.0208, 0.011),
        (0.0200, 0.018),
    ]
    inner_profile = [
        (0.0176, 0.000),
        (0.0176, 0.004),
        (0.0172, 0.011),
        (0.0168, 0.018),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
        "dish_soap_pump_collar",
    )


def _build_annular_shell(outer_radius: float, inner_radius: float, length: float, name: str):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, length)],
            [(inner_radius, 0.0), (inner_radius, length)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        name,
    )


def _build_actuator_pad(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height,
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dish_soap_pump_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.53, 0.78, 0.40, 0.68))
    pump_white = model.material("pump_white", rgba=(0.95, 0.95, 0.93, 1.0))
    pump_accent = model.material("pump_accent", rgba=(0.84, 0.84, 0.82, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        _build_bottle_shell(),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle_body.visual(
        _build_annular_shell(0.0215, 0.0170, 0.004, "bottle_neck_seat_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.1413)),
        material=bottle_plastic,
        name="neck_seat",
    )
    bottle_body.visual(
        _build_annular_shell(0.0170, 0.0110, 0.018, "bottle_threaded_neck"),
        origin=Origin(xyz=(0.0, 0.0, 0.1380)),
        material=bottle_plastic,
        name="bottle_neck",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.048, length=0.156),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
    )

    pump_collar = model.part("pump_collar")
    pump_collar.visual(
        _build_collar_shell(),
        material=pump_accent,
        name="collar_shell",
    )
    pump_collar.visual(
        _build_annular_shell(0.0215, 0.0176, 0.002, "pump_collar_base_ring"),
        origin=Origin(),
        material=pump_accent,
        name="collar_base_flange",
    )
    for post_name, post_origin, rib_size, rib_origin in (
        (
            "collar_post_east",
            (0.0245, 0.0, 0.009),
            (0.007, 0.006, 0.006),
            (0.0225, 0.0, 0.006),
        ),
        (
            "collar_post_west",
            (-0.0245, 0.0, 0.009),
            (0.007, 0.006, 0.006),
            (-0.0225, 0.0, 0.006),
        ),
        (
            "collar_post_north",
            (0.0, 0.0245, 0.009),
            (0.006, 0.007, 0.006),
            (0.0, 0.0225, 0.006),
        ),
        (
            "collar_post_south",
            (0.0, -0.0245, 0.009),
            (0.006, 0.007, 0.006),
            (0.0, -0.0225, 0.006),
        ),
    ):
        pump_collar.visual(
            Cylinder(radius=0.003, length=0.010),
            origin=Origin(xyz=post_origin),
            material=pump_accent,
            name=post_name,
        )
        pump_collar.visual(
            Box(rib_size),
            origin=Origin(xyz=rib_origin),
            material=pump_accent,
            name=f"{post_name}_rib",
        )
    pump_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0215, length=0.018),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )
    model.articulation(
        "body_to_pump_collar",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=pump_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.1453)),
    )

    actuator_cap = model.part("actuator_cap")
    actuator_cap.visual(
        Cylinder(radius=0.0075, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=pump_white,
        name="plunger_stem",
    )
    for guide_name, guide_origin in (
        ("guide_east", (0.0245, 0.0, -0.003)),
        ("guide_west", (-0.0245, 0.0, -0.003)),
        ("guide_north", (0.0, 0.0245, -0.003)),
        ("guide_south", (0.0, -0.0245, -0.003)),
    ):
        actuator_cap.visual(
            Cylinder(radius=0.003, length=0.014),
            origin=Origin(xyz=guide_origin),
            material=pump_white,
            name=guide_name,
        )
    actuator_cap.visual(
        _build_actuator_pad(0.050, 0.044, 0.010, 0.010, "actuator_button_pad"),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=pump_white,
        name="actuator_button",
    )
    actuator_cap.visual(
        _build_actuator_pad(0.038, 0.028, 0.004, 0.008, "actuator_top_pad_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=pump_white,
        name="actuator_top_pad",
    )
    actuator_cap.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.042)),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "collar_to_actuator_cap",
        ArticulationType.PRISMATIC,
        parent=pump_collar,
        child=actuator_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=pump_white,
        name="nozzle_hub",
    )
    nozzle.visual(
        Cylinder(radius=0.0045, length=0.032),
        origin=Origin(xyz=(0.020, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_white,
        name="spout_body",
    )
    nozzle.visual(
        Cylinder(radius=0.0032, length=0.012),
        origin=Origin(xyz=(0.041, 0.0, 0.003), rpy=(0.0, math.radians(112.0), 0.0)),
        material=pump_white,
        name="spout_tip",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.056, 0.014, 0.022)),
        mass=0.03,
        origin=Origin(xyz=(0.020, 0.0, 0.007)),
    )
    model.articulation(
        "cap_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=actuator_cap,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=-math.radians(100.0),
            upper=math.radians(100.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle_body = object_model.get_part("bottle_body")
    pump_collar = object_model.get_part("pump_collar")
    actuator_cap = object_model.get_part("actuator_cap")
    nozzle = object_model.get_part("nozzle")
    cap_slide = object_model.get_articulation("collar_to_actuator_cap")
    nozzle_pivot = object_model.get_articulation("cap_to_nozzle")

    cap_travel = 0.006
    if cap_slide.motion_limits is not None and cap_slide.motion_limits.upper is not None:
        cap_travel = cap_slide.motion_limits.upper

    nozzle_swivel = math.radians(80.0)
    if nozzle_pivot.motion_limits is not None and nozzle_pivot.motion_limits.upper is not None:
        nozzle_swivel = min(nozzle_swivel, nozzle_pivot.motion_limits.upper)

    ctx.expect_overlap(
        pump_collar,
        bottle_body,
        axes="xy",
        elem_a="collar_shell",
        elem_b="bottle_shell",
        min_overlap=0.030,
        name="pump collar stays centered over the bottle neck",
    )

    with ctx.pose({cap_slide: 0.0}):
        ctx.expect_gap(
            actuator_cap,
            pump_collar,
            axis="z",
            positive_elem="actuator_button",
            negative_elem="collar_shell",
            min_gap=0.005,
            max_gap=0.0075,
            name="resting actuator button sits slightly above the collar",
        )
        ctx.expect_contact(
            nozzle,
            actuator_cap,
            elem_a="nozzle_hub",
            elem_b="actuator_top_pad",
            name="nozzle hub sits on the actuator cap",
        )
        cap_rest = ctx.part_world_position(actuator_cap)
        nozzle_rest_aabb = ctx.part_world_aabb(nozzle)

    with ctx.pose({cap_slide: cap_travel}):
        ctx.expect_gap(
            actuator_cap,
            pump_collar,
            axis="z",
            positive_elem="actuator_button",
            negative_elem="collar_shell",
            min_gap=0.0,
            max_gap=0.0015,
            name="pressed actuator nearly closes the collar gap",
        )
        cap_pressed = ctx.part_world_position(actuator_cap)

    ctx.check(
        "actuator cap moves downward when pressed",
        cap_rest is not None
        and cap_pressed is not None
        and cap_pressed[2] < cap_rest[2] - 0.005,
        details=f"rest={cap_rest}, pressed={cap_pressed}",
    )

    with ctx.pose({nozzle_pivot: nozzle_swivel}):
        ctx.expect_contact(
            nozzle,
            actuator_cap,
            elem_a="nozzle_hub",
            elem_b="actuator_top_pad",
            name="nozzle hub remains seated while swiveling",
        )
        nozzle_swiveled_aabb = ctx.part_world_aabb(nozzle)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    nozzle_rest_center = _aabb_center(nozzle_rest_aabb)
    nozzle_swiveled_center = _aabb_center(nozzle_swiveled_aabb)
    ctx.check(
        "nozzle swings sideways around its pivot",
        nozzle_rest_center is not None
        and nozzle_swiveled_center is not None
        and nozzle_swiveled_center[1] > nozzle_rest_center[1] + 0.010,
        details=f"rest={nozzle_rest_center}, swiveled={nozzle_swiveled_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
