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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HANDLE_LENGTH = 0.135
HANDLE_WIDTH = 0.026
HANDLE_HEIGHT = 0.024
SIDE_PANEL_THICKNESS = 0.002


def _xz_plate_mesh(
    profile: list[tuple[float, float]],
    thickness: float,
    name: str,
):
    geom = ExtrudeGeometry(profile, thickness, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _blade_mesh(name: str):
    blade_profile = [
        (0.012, -0.0050),
        (0.042, -0.0050),
        (0.060, 0.0000),
        (0.042, 0.0050),
        (0.020, 0.0050),
        (0.012, 0.0015),
    ]
    return _xz_plate_mesh(blade_profile, 0.0012, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_knife")

    handle_orange = model.material("handle_orange", rgba=(0.93, 0.50, 0.12, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("blade_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    slider_black = model.material("slider_black", rgba=(0.11, 0.11, 0.12, 1.0))

    handle = model.part("handle")

    side_profile = [
        (-0.066, 0.003),
        (-0.060, 0.000),
        (-0.010, 0.000),
        (0.025, 0.0015),
        (0.050, 0.0035),
        (0.061, 0.0060),
        (0.067, 0.0100),
        (0.067, 0.0160),
        (0.060, 0.0200),
        (0.030, 0.0240),
        (-0.020, 0.0240),
        (-0.052, 0.0210),
        (-0.065, 0.0160),
        (-0.066, 0.0100),
    ]
    side_mesh = _xz_plate_mesh(side_profile, SIDE_PANEL_THICKNESS, "handle_side_panel")
    handle.visual(
        side_mesh,
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=handle_orange,
        name="left_shell",
    )
    handle.visual(
        side_mesh,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=handle_orange,
        name="right_shell",
    )
    handle.visual(
        Box((0.106, 0.022, 0.004)),
        origin=Origin(xyz=(-0.004, 0.0, 0.002)),
        material=handle_dark,
        name="bottom_rail",
    )
    handle.visual(
        Box((0.080, 0.006, 0.004)),
        origin=Origin(xyz=(-0.012, 0.008, 0.022)),
        material=handle_dark,
        name="left_top_rail",
    )
    handle.visual(
        Box((0.080, 0.006, 0.004)),
        origin=Origin(xyz=(-0.012, -0.008, 0.022)),
        material=handle_dark,
        name="right_top_rail",
    )
    handle.visual(
        Box((0.006, 0.022, 0.017)),
        origin=Origin(xyz=(-0.062, 0.0, 0.0105)),
        material=handle_dark,
        name="rear_cap",
    )
    handle.visual(
        Box((0.012, 0.022, 0.004)),
        origin=Origin(xyz=(0.060, 0.0, 0.002)),
        material=handle_dark,
        name="nose_lower_lip",
    )
    handle.visual(
        Box((0.010, 0.022, 0.004)),
        origin=Origin(xyz=(0.058, 0.0, 0.018)),
        material=handle_dark,
        name="nose_upper_lip",
    )
    handle.visual(
        Box((0.012, 0.004, 0.012)),
        origin=Origin(xyz=(0.060, 0.009, 0.010)),
        material=handle_dark,
        name="left_nose_cheek",
    )
    handle.visual(
        Box((0.012, 0.004, 0.012)),
        origin=Origin(xyz=(0.060, -0.009, 0.010)),
        material=handle_dark,
        name="right_nose_cheek",
    )
    handle.visual(
        Box((0.005, 0.018, 0.005)),
        origin=Origin(xyz=(0.0505, 0.0, 0.0205)),
        material=handle_dark,
        name="guard_hinge_mount",
    )
    handle.inertial = Inertial.from_geometry(
        Box((HANDLE_LENGTH, HANDLE_WIDTH, HANDLE_HEIGHT)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_HEIGHT / 2.0)),
    )

    blade_carriage = model.part("blade_carriage")
    blade_carriage.visual(
        Box((0.030, 0.014, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, -0.004)),
        material=handle_dark,
        name="carriage_block",
    )
    blade_carriage.visual(
        Box((0.006, 0.004, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.006)),
        material=handle_dark,
        name="slider_stem",
    )
    blade_carriage.visual(
        Box((0.014, 0.010, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.014)),
        material=slider_black,
        name="slider_button",
    )
    blade_carriage.visual(
        _blade_mesh("knife_blade"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="blade",
    )
    blade_carriage.inertial = Inertial.from_geometry(
        Box((0.060, 0.014, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(0.028, 0.0, 0.002)),
    )

    guard_flap = model.part("guard_flap")
    guard_flap.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_dark,
        name="hinge_barrel",
    )
    guard_flap.visual(
        Box((0.016, 0.018, 0.005)),
        origin=Origin(xyz=(0.008, 0.0, -0.0025)),
        material=handle_dark,
        name="flap_panel",
    )
    guard_flap.visual(
        Box((0.003, 0.018, 0.006)),
        origin=Origin(xyz=(0.0165, 0.0, -0.0050)),
        material=handle_dark,
        name="flap_lip",
    )
    guard_flap.inertial = Inertial.from_geometry(
        Box((0.020, 0.018, 0.010)),
        mass=0.02,
        origin=Origin(xyz=(0.008, 0.0, -0.002)),
    )

    slide_joint = model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carriage,
        origin=Origin(xyz=(-0.007, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.028,
        ),
    )

    model.articulation(
        "handle_to_guard_flap",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=guard_flap,
        origin=Origin(xyz=(0.0555, 0.0, 0.0205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(75.0),
        ),
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

    handle = object_model.get_part("handle")
    blade_carriage = object_model.get_part("blade_carriage")
    guard_flap = object_model.get_part("guard_flap")
    slide_joint = object_model.get_articulation("handle_to_blade_carriage")
    flap_joint = object_model.get_articulation("handle_to_guard_flap")

    ctx.expect_within(
        blade_carriage,
        handle,
        axes="y",
        inner_elem="carriage_block",
        margin=0.001,
        name="carriage stays centered between handle side shells",
    )
    ctx.expect_overlap(
        blade_carriage,
        handle,
        axes="x",
        elem_a="carriage_block",
        min_overlap=0.025,
        name="carriage remains inserted inside the handle at rest",
    )
    ctx.expect_overlap(
        guard_flap,
        handle,
        axes="y",
        elem_a="flap_panel",
        min_overlap=0.012,
        name="guard flap spans the width of the nose opening",
    )

    handle_aabb = ctx.part_world_aabb(handle)
    blade_retracted = ctx.part_element_world_aabb(blade_carriage, elem="blade")
    ctx.check(
        "blade is retracted behind the handle nose at rest",
        handle_aabb is not None
        and blade_retracted is not None
        and blade_retracted[1][0] < handle_aabb[1][0] - 0.006,
        details=f"handle_aabb={handle_aabb}, blade_retracted={blade_retracted}",
    )

    rest_pos = ctx.part_world_position(blade_carriage)
    with ctx.pose({slide_joint: 0.028}):
        extended_pos = ctx.part_world_position(blade_carriage)
        blade_extended = ctx.part_element_world_aabb(blade_carriage, elem="blade")
        ctx.expect_overlap(
            blade_carriage,
            handle,
            axes="x",
            elem_a="carriage_block",
            min_overlap=0.010,
            name="carriage retains insertion at full blade extension",
        )
        ctx.check(
            "blade extends forward when the carriage slides",
            handle_aabb is not None
            and blade_extended is not None
            and blade_extended[1][0] > handle_aabb[1][0] + 0.010,
            details=f"handle_aabb={handle_aabb}, blade_extended={blade_extended}",
        )
        ctx.check(
            "blade carriage moves toward the nose",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.020,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    flap_closed = ctx.part_element_world_aabb(guard_flap, elem="flap_panel")
    with ctx.pose({flap_joint: math.radians(75.0)}):
        flap_open = ctx.part_element_world_aabb(guard_flap, elem="flap_panel")
        ctx.check(
            "guard flap rotates upward out of the nose opening",
            flap_closed is not None
            and flap_open is not None
            and flap_open[1][2] > flap_closed[1][2] + 0.010,
            details=f"closed={flap_closed}, open={flap_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
