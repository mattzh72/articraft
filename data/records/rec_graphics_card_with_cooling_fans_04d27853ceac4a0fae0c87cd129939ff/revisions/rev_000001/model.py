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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="liquid_cooled_graphics_card")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.22, 0.12, 1.0))
    shroud_black = model.material("shroud_black", rgba=(0.10, 0.11, 0.12, 1.0))
    pump_black = model.material("pump_black", rgba=(0.16, 0.17, 0.18, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.30, 0.31, 0.33, 1.0))
    brushed_silver = model.material("brushed_silver", rgba=(0.72, 0.74, 0.77, 1.0))

    board_length = 0.285
    board_width = 0.112
    board_thickness = 0.0024
    shroud_length = 0.248
    shroud_width = 0.092
    shroud_height = 0.019
    shroud_center_x = 0.010
    shroud_top_z = board_thickness + shroud_height

    pod_center = (0.116, 0.041, shroud_top_z)
    pod_outer_radius = 0.029
    pod_inner_radius = 0.0215
    pod_height = 0.010

    body = model.part("card_body")
    body.visual(
        Box((board_length, board_width, board_thickness)),
        origin=Origin(xyz=(0.0, 0.0, board_thickness * 0.5)),
        material=pcb_green,
        name="pcb_board",
    )

    shroud_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(shroud_length, shroud_width, 0.010, corner_segments=10),
            shroud_height,
            center=True,
            cap=True,
            closed=True,
        ),
        "gpu_main_shroud",
    )
    body.visual(
        shroud_mesh,
        origin=Origin(xyz=(shroud_center_x, 0.0, board_thickness + shroud_height * 0.5)),
        material=shroud_black,
        name="main_shroud",
    )

    body.visual(
        Box((0.114, 0.018, 0.006)),
        origin=Origin(xyz=(0.016, -0.028, board_thickness + 0.003)),
        material=metal_dark,
        name="lower_side_rail",
    )

    body.visual(
        Box((0.002, 0.118, 0.040)),
        origin=Origin(xyz=(-board_length * 0.5 - 0.001, 0.0, 0.020)),
        material=brushed_silver,
        name="io_bracket",
    )
    body.visual(
        Box((0.010, 0.018, 0.010)),
        origin=Origin(xyz=(-board_length * 0.5 - 0.004, -0.036, 0.008)),
        material=brushed_silver,
        name="pcie_tab",
    )

    pump_cover_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.088, 0.056, 0.008, corner_segments=10),
            0.010,
            center=True,
            cap=True,
            closed=True,
        ),
        "gpu_pump_cover",
    )
    body.visual(
        pump_cover_mesh,
        origin=Origin(xyz=(-0.014, 0.001, shroud_top_z + 0.005)),
        material=pump_black,
        name="pump_block_cover",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.0022),
        origin=Origin(xyz=(-0.014, 0.001, shroud_top_z + 0.0111)),
        material=metal_dark,
        name="pump_cap",
    )

    body.visual(
        Box((0.128, 0.012, 0.002)),
        origin=Origin(xyz=(0.040, 0.012, shroud_top_z + 0.001)),
        material=brushed_silver,
        name="top_accent",
    )

    pod_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(pod_outer_radius, segments=56),
            [_circle_profile(pod_inner_radius, segments=56)],
            pod_height,
            center=True,
            cap=True,
            closed=True,
        ),
        "gpu_assist_pod_ring",
    )
    body.visual(
        pod_ring_mesh,
        origin=Origin(
            xyz=(
                pod_center[0],
                pod_center[1],
                shroud_top_z + pod_height * 0.5,
            )
        ),
        material=shroud_black,
        name="assist_pod_ring",
    )
    body.visual(
        Cylinder(radius=pod_inner_radius - 0.0025, length=0.0018),
        origin=Origin(xyz=(pod_center[0], pod_center[1], shroud_top_z + 0.0009)),
        material=metal_dark,
        name="pod_backplate",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.0024),
        origin=Origin(xyz=(pod_center[0], pod_center[1], shroud_top_z + 0.0018)),
        material=brushed_silver,
        name="pod_spindle",
    )

    body.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(
            xyz=(0.030, -shroud_width * 0.5 - 0.002, board_thickness + 0.010),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=metal_dark,
        name="selector_mount",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.295, 0.120, 0.042)),
        mass=1.65,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    assist_fan = model.part("assist_fan")
    assist_fan.visual(
        Cylinder(radius=0.0065, length=0.005),
        material=metal_dark,
        name="rotor_hub",
    )
    blade_names = ["blade_0", "blade_1", "blade_2", "blade_3", "blade_4"]
    for blade_index, blade_name in enumerate(blade_names):
        angle = blade_index * (2.0 * math.pi / 5.0) + 0.38
        assist_fan.visual(
            Box((0.020, 0.007, 0.003)),
            origin=Origin(
                xyz=(0.009, 0.0, 0.0),
                rpy=(0.24, 0.0, angle),
            ),
            material=shroud_black,
            name=blade_name,
        )
    assist_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.006),
        mass=0.045,
        origin=Origin(),
    )

    model.articulation(
        "body_to_assist_fan",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=assist_fan,
        origin=Origin(xyz=(pod_center[0], pod_center[1], shroud_top_z + 0.0055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=40.0),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.0034, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=metal_dark,
        name="selector_shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.0085, length=0.008),
        origin=Origin(xyz=(0.0, -0.0095, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=pump_black,
        name="selector_body",
    )
    selector_knob.visual(
        Box((0.0024, 0.006, 0.0034)),
        origin=Origin(xyz=(0.0, -0.013, 0.0052)),
        material=brushed_silver,
        name="selector_pointer",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0085, length=0.014),
        mass=0.018,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.030, -shroud_width * 0.5 - 0.004, board_thickness + 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.10, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("card_body")
    fan = object_model.get_part("assist_fan")
    knob = object_model.get_part("selector_knob")
    fan_joint = object_model.get_articulation("body_to_assist_fan")
    knob_joint = object_model.get_articulation("body_to_selector_knob")

    ctx.check(
        "critical gpu parts exist",
        all(part is not None for part in (body, fan, knob)),
        details="card_body, assist_fan, and selector_knob should all be authored.",
    )

    ctx.expect_gap(
        fan,
        body,
        axis="z",
        positive_elem="rotor_hub",
        negative_elem="pod_backplate",
        min_gap=0.0002,
        max_gap=0.004,
        name="assist fan rotor clears the pod backplate",
    )
    ctx.expect_overlap(
        fan,
        body,
        axes="xy",
        elem_a="rotor_hub",
        elem_b="assist_pod_ring",
        min_overlap=0.012,
        name="assist fan stays centered in the circular pod footprint",
    )
    ctx.expect_contact(
        fan,
        body,
        elem_a="rotor_hub",
        elem_b="pod_spindle",
        name="assist fan is supported on the central spindle",
    )
    ctx.expect_contact(
        knob,
        body,
        elem_a="selector_shaft",
        elem_b="selector_mount",
        name="selector knob seats on its short shaft mount",
    )

    fan_blade_rest = _aabb_center(ctx.part_element_world_aabb(fan, elem="blade_0"))
    with ctx.pose({fan_joint: math.pi * 0.5}):
        fan_blade_turned = _aabb_center(ctx.part_element_world_aabb(fan, elem="blade_0"))
    fan_ok = (
        fan_blade_rest is not None
        and fan_blade_turned is not None
        and abs(fan_blade_rest[2] - fan_blade_turned[2]) < 0.002
        and math.hypot(
            fan_blade_rest[0] - fan_blade_turned[0],
            fan_blade_rest[1] - fan_blade_turned[1],
        )
        > 0.006
    )
    ctx.check(
        "assist fan rotates about its central z axis",
        fan_ok,
        details=f"rest={fan_blade_rest}, turned={fan_blade_turned}",
    )

    knob_pointer_rest = _aabb_center(ctx.part_element_world_aabb(knob, elem="selector_pointer"))
    with ctx.pose({knob_joint: 1.2}):
        knob_pointer_turned = _aabb_center(ctx.part_element_world_aabb(knob, elem="selector_pointer"))
    knob_ok = (
        knob_pointer_rest is not None
        and knob_pointer_turned is not None
        and abs(knob_pointer_rest[1] - knob_pointer_turned[1]) < 0.002
        and math.hypot(
            knob_pointer_rest[0] - knob_pointer_turned[0],
            knob_pointer_rest[2] - knob_pointer_turned[2],
        )
        > 0.003
    )
    ctx.check(
        "selector knob rotates on its own short shaft",
        knob_ok,
        details=f"rest={knob_pointer_rest}, turned={knob_pointer_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
