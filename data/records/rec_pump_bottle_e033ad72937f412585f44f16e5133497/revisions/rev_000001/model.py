from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_bottle_shell():
    outer_profile = [
        (0.024, 0.000),
        (0.0272, 0.006),
        (0.0272, 0.128),
        (0.0260, 0.138),
        (0.0220, 0.149),
        (0.0180, 0.156),
        (0.0210, 0.160),
    ]
    inner_profile = [
        (0.000, 0.0035),
        (0.0240, 0.0075),
        (0.0240, 0.127),
        (0.0228, 0.137),
        (0.0193, 0.147),
        (0.0158, 0.155),
        (0.0130, 0.160),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "wash_bottle_shell",
    )


def _build_collar_shell():
    outer_profile = [
        (0.0240, 0.000),
        (0.0240, 0.010),
        (0.0220, 0.018),
        (0.0195, 0.024),
    ]
    inner_profile = [
        (0.0130, 0.000),
        (0.0138, 0.010),
        (0.0138, 0.018),
        (0.0128, 0.024),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "pump_collar_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_wash_pump_bottle")

    bottle_poly = model.material("bottle_poly", rgba=(0.88, 0.91, 0.92, 1.0))
    cap_plastic = model.material("cap_plastic", rgba=(0.84, 0.84, 0.82, 1.0))
    plunger_plastic = model.material("plunger_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    dark_accent = model.material("dark_accent", rgba=(0.18, 0.21, 0.24, 1.0))
    tab_accent = model.material("tab_accent", rgba=(0.28, 0.39, 0.49, 1.0))

    bottle = model.part("bottle")
    bottle.visual(_build_bottle_shell(), material=bottle_poly, name="bottle_shell")
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.160),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    collar = model.part("pump_collar")
    collar.visual(_build_collar_shell(), material=cap_plastic, name="collar_shell")
    collar.visual(
        Box((0.0070, 0.0080, 0.0090)),
        origin=Origin(xyz=(0.0210, 0.0, 0.0150)),
        material=cap_plastic,
        name="hinge_mount",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.024),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0112, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=plunger_plastic,
        name="plunger_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0102, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=dark_accent,
        name="pump_piston",
    )
    plunger.visual(
        Cylinder(radius=0.0134, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=dark_accent,
        name="plunger_flange",
    )
    plunger.visual(
        Cylinder(radius=0.0160, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0130)),
        material=plunger_plastic,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.0, -0.014, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plunger_plastic,
        name="plunger_nozzle",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.062),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
    )

    locking_tab = model.part("locking_tab")
    locking_tab.visual(
        Cylinder(radius=0.0022, length=0.006),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tab_accent,
        name="tab_barrel",
    )
    locking_tab.visual(
        Box((0.0070, 0.0040, 0.0140)),
        origin=Origin(xyz=(0.0040, 0.0, -0.0070)),
        material=tab_accent,
        name="tab_body",
    )
    locking_tab.visual(
        Box((0.0040, 0.0040, 0.0040)),
        origin=Origin(xyz=(0.0060, 0.0, -0.0150)),
        material=tab_accent,
        name="tab_tip",
    )
    locking_tab.inertial = Inertial.from_geometry(
        Box((0.010, 0.006, 0.018)),
        mass=0.01,
        origin=Origin(xyz=(0.004, 0.0, -0.008)),
    )

    model.articulation(
        "bottle_to_pump_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )
    model.articulation(
        "pump_collar_to_plunger",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.15,
            lower=0.0,
            upper=0.010,
        ),
    )
    model.articulation(
        "pump_collar_to_locking_tab",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=locking_tab,
        origin=Origin(xyz=(0.0240, 0.0, 0.016)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=0.75,
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
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("pump_collar")
    plunger = object_model.get_part("plunger")
    locking_tab = object_model.get_part("locking_tab")

    plunger_joint = object_model.get_articulation("pump_collar_to_plunger")
    tab_joint = object_model.get_articulation("pump_collar_to_locking_tab")

    ctx.expect_overlap(
        collar,
        bottle,
        axes="xy",
        min_overlap=0.030,
        name="pump assembly stays centered over the bottle neck",
    )
    ctx.expect_contact(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        contact_tol=0.0005,
        name="pump collar seats on the bottle mouth",
    )

    with ctx.pose({plunger_joint: 0.0}):
        ctx.expect_contact(
            plunger,
            collar,
            elem_a="plunger_flange",
            elem_b="collar_shell",
            contact_tol=0.0005,
            name="plunger flange rests on the collar at full extension",
        )
        ctx.expect_within(
            plunger,
            collar,
            axes="xy",
            inner_elem="plunger_stem",
            outer_elem="collar_shell",
            margin=0.0008,
            name="plunger stem is guided within the collar at rest",
        )
        ctx.expect_overlap(
            plunger,
            collar,
            axes="z",
            elem_a="plunger_stem",
            elem_b="collar_shell",
            min_overlap=0.0235,
            name="plunger retains insertion in the collar at rest",
        )
        rest_plunger_pos = ctx.part_world_position(plunger)

    with ctx.pose({plunger_joint: 0.010}):
        ctx.expect_within(
            plunger,
            collar,
            axes="xy",
            inner_elem="plunger_stem",
            outer_elem="collar_shell",
            margin=0.0008,
            name="plunger stem stays aligned while depressed",
        )
        ctx.expect_overlap(
            plunger,
            collar,
            axes="z",
            elem_a="plunger_stem",
            elem_b="collar_shell",
            min_overlap=0.018,
            name="plunger remains captured when depressed",
        )
        depressed_plunger_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger travels downward along the bottle axis",
        rest_plunger_pos is not None
        and depressed_plunger_pos is not None
        and depressed_plunger_pos[2] < rest_plunger_pos[2] - 0.008,
        details=f"rest={rest_plunger_pos}, depressed={depressed_plunger_pos}",
    )

    with ctx.pose({tab_joint: 0.0}):
        closed_tip_aabb = ctx.part_element_world_aabb(locking_tab, elem="tab_tip")

    with ctx.pose({tab_joint: 0.75}):
        open_tip_aabb = ctx.part_element_world_aabb(locking_tab, elem="tab_tip")

    ctx.check(
        "locking tab swings outward from the collar side",
        closed_tip_aabb is not None
        and open_tip_aabb is not None
        and open_tip_aabb[1][0] > closed_tip_aabb[1][0] + 0.004,
        details=f"closed={closed_tip_aabb}, open={open_tip_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
