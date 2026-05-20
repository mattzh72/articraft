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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _build_bottle_shell():
    outer_profile = [
        (0.012, 0.000),
        (0.036, 0.004),
        (0.038, 0.020),
        (0.038, 0.155),
        (0.034, 0.173),
        (0.024, 0.184),
        (0.018, 0.189),
        (0.0155, 0.193),
        (0.0155, 0.198),
        (0.015, 0.205),
    ]
    inner_profile = [
        (0.000, 0.003),
        (0.0335, 0.008),
        (0.0350, 0.020),
        (0.0350, 0.154),
        (0.0310, 0.171),
        (0.0215, 0.182),
        (0.0155, 0.188),
        (0.0115, 0.193),
        (0.0110, 0.198),
        (0.0105, 0.205),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_collar_shell():
    outer_profile = [
        (0.0220, 0.000),
        (0.0220, 0.021),
        (0.0205, 0.030),
    ]
    inner_profile = [
        (0.0160, 0.000),
        (0.0160, 0.024),
        (0.0149, 0.030),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_plunger_head():
    head_profile = rounded_rect_profile(0.058, 0.034, 0.010, corner_segments=8)
    return ExtrudeGeometry(head_profile, 0.012, cap=True, center=True).translate(0.002, 0.0, 0.004)


def _build_spout_mesh():
    pivot_sleeve = LatheGeometry.from_shell_profiles(
        [
            (0.0066, -0.0055),
            (0.0066, 0.0055),
        ],
        [
            (0.0047, -0.0055),
            (0.0047, 0.0055),
        ],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )
    nozzle = tube_from_spline_points(
        [
            (0.0075, 0.0, 0.0005),
            (0.0220, 0.0, 0.0018),
            (0.0380, 0.0, 0.0008),
            (0.0500, 0.0, -0.0018),
        ],
        radius=0.0028,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    bridge = CylinderGeometry(radius=0.0031, height=0.007).rotate_y(math.pi / 2.0).translate(
        0.0068,
        0.0,
        0.0005,
    )
    pivot_sleeve.merge(nozzle)
    pivot_sleeve.merge(bridge)
    return pivot_sleeve


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lotion_pump_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.92, 0.88, 0.80, 1.0))
    pump_white = model.material("pump_white", rgba=(0.96, 0.96, 0.95, 1.0))
    pump_gray = model.material("pump_gray", rgba=(0.84, 0.85, 0.86, 1.0))

    bottle = model.part("bottle_body")
    bottle.visual(
        mesh_from_geometry(_build_bottle_shell(), "bottle_shell"),
        material=bottle_plastic,
        name="elem_body_shell",
    )
    for idx, z_center in enumerate((0.193, 0.197, 0.201), start=1):
        bottle.visual(
            Cylinder(radius=0.0160, length=0.0026),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=bottle_plastic,
            name=f"elem_thread_ring_{idx}",
        )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.039, length=0.205),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
    )

    collar = model.part("pump_collar")
    collar.visual(
        mesh_from_geometry(_build_collar_shell(), "pump_collar_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.19395)),
        material=pump_gray,
        name="elem_collar_shell",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.030),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.209)),
    )

    plunger = model.part("pump_plunger")
    plunger.visual(
        Cylinder(radius=0.0080, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=pump_white,
        name="elem_plunger_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0110, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=pump_white,
        name="elem_plunger_sleeve",
    )
    plunger.visual(
        mesh_from_geometry(_build_plunger_head(), "pump_head"),
        material=pump_white,
        name="elem_plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0044, length=0.011),
        origin=Origin(xyz=(0.028, 0.0, 0.0155)),
        material=pump_white,
        name="elem_spout_pivot_boss",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.070, 0.036, 0.070)),
        mass=0.06,
        origin=Origin(xyz=(0.004, 0.0, -0.010)),
    )

    spout = model.part("dispensing_spout")
    spout.visual(
        mesh_from_geometry(_build_spout_mesh(), "dispensing_spout"),
        material=pump_white,
        name="elem_spout_nozzle",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.050, 0.015, 0.015)),
        mass=0.02,
        origin=Origin(xyz=(0.020, 0.0, 0.000)),
    )

    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(),
    )

    model.articulation(
        "collar_to_plunger",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.236)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.010,
        ),
    )

    model.articulation(
        "plunger_to_spout",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=spout,
        origin=Origin(xyz=(0.028, 0.0, 0.0155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-1.30,
            upper=1.30,
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

    bottle = object_model.get_part("bottle_body")
    collar = object_model.get_part("pump_collar")
    plunger = object_model.get_part("pump_plunger")
    spout = object_model.get_part("dispensing_spout")
    pump_slide = object_model.get_articulation("collar_to_plunger")
    spout_swivel = object_model.get_articulation("plunger_to_spout")

    ctx.check(
        "all pump parts exist",
        all(part is not None for part in (bottle, collar, plunger, spout)),
        details="Expected bottle_body, pump_collar, pump_plunger, and dispensing_spout.",
    )
    ctx.check(
        "plunger axis points down the bottle neck",
        tuple(pump_slide.axis) == (0.0, 0.0, -1.0),
        details=f"axis={pump_slide.axis}",
    )
    ctx.check(
        "spout swivel axis is vertical",
        tuple(spout_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spout_swivel.axis}",
    )

    with ctx.pose({pump_slide: 0.0, spout_swivel: 0.0}):
        ctx.expect_overlap(
            collar,
            bottle,
            axes="xy",
            elem_a="elem_collar_shell",
            elem_b="elem_body_shell",
            min_overlap=0.030,
            name="pump collar sits over the bottle neck footprint",
        )
        ctx.expect_within(
            plunger,
            collar,
            axes="xy",
            inner_elem="elem_plunger_sleeve",
            outer_elem="elem_collar_shell",
            margin=0.0008,
            name="plunger sleeve stays centered within the collar",
        )
        ctx.expect_overlap(
            plunger,
            collar,
            axes="z",
            elem_a="elem_plunger_sleeve",
            elem_b="elem_collar_shell",
            min_overlap=0.005,
            name="resting plunger sleeve remains inserted in the collar",
        )

    rest_plunger_pos = ctx.part_world_position(plunger)
    slide_upper = pump_slide.motion_limits.upper if pump_slide.motion_limits is not None else None
    with ctx.pose({pump_slide: slide_upper, spout_swivel: 0.0}):
        ctx.expect_within(
            plunger,
            collar,
            axes="xy",
            inner_elem="elem_plunger_sleeve",
            outer_elem="elem_collar_shell",
            margin=0.0008,
            name="pressed plunger sleeve stays centered within the collar",
        )
        ctx.expect_overlap(
            plunger,
            collar,
            axes="z",
            elem_a="elem_plunger_sleeve",
            elem_b="elem_collar_shell",
            min_overlap=0.012,
            name="pressed plunger sleeve remains guided by the collar",
        )
        pressed_plunger_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger presses downward",
        rest_plunger_pos is not None
        and pressed_plunger_pos is not None
        and pressed_plunger_pos[2] < rest_plunger_pos[2] - 0.008,
        details=f"rest={rest_plunger_pos}, pressed={pressed_plunger_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({pump_slide: 0.0, spout_swivel: 0.0}):
        rest_nozzle_center = _aabb_center(
            ctx.part_element_world_aabb(spout, elem="elem_spout_nozzle")
        )
    with ctx.pose({pump_slide: 0.0, spout_swivel: 1.15}):
        turned_nozzle_center = _aabb_center(
            ctx.part_element_world_aabb(spout, elem="elem_spout_nozzle")
        )

    ctx.check(
        "spout points forward at rest and swings sideways when rotated",
        rest_nozzle_center is not None
        and turned_nozzle_center is not None
        and rest_nozzle_center[0] > 0.045
        and abs(rest_nozzle_center[1]) < 0.004
        and turned_nozzle_center[1] > 0.018
        and abs(turned_nozzle_center[2] - rest_nozzle_center[2]) < 0.002,
        details=f"rest={rest_nozzle_center}, turned={turned_nozzle_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
