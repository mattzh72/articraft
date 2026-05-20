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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_punch")

    shell = model.material("shell", rgba=(0.17, 0.17, 0.19, 1.0))
    lever_metal = model.material("lever_metal", rgba=(0.74, 0.75, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.84, 0.85, 0.87, 1.0))

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
        *,
        bottom: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        z_shift = bottom + height / 2.0
        return [(x, y, z + z_shift) for y, z in rounded_rect_profile(width, height, radius)]

    body = model.part("body")
    body_shell = mesh_from_geometry(
        section_loft(
            [
                yz_section(-0.056, 0.056, 0.014, 0.006),
                yz_section(-0.030, 0.060, 0.018, 0.008),
                yz_section(0.000, 0.062, 0.021, 0.010),
                yz_section(0.030, 0.060, 0.020, 0.010),
                yz_section(0.056, 0.052, 0.016, 0.007),
            ]
        ),
        "body_shell",
    )
    body.visual(body_shell, material=shell, name="shell")
    body.visual(
        Box((0.020, 0.024, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.012)),
        material=shell,
        name="die_housing",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.010, 0.0, 0.009)),
        material=shell,
        name="die_sleeve",
    )
    body.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(-0.045, -0.024, 0.023)),
        material=shell,
        name="rear_support_0",
    )
    body.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(-0.045, 0.024, 0.023)),
        material=shell,
        name="rear_support_1",
    )
    body.visual(
        Box((0.012, 0.016, 0.012)),
        origin=Origin(xyz=(-0.052, 0.0, 0.018)),
        material=shell,
        name="tab_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.115, 0.065, 0.038)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    lever = model.part("lever")
    lever_shell = mesh_from_geometry(
        section_loft(
            [
                yz_section(-0.006, 0.026, 0.010, 0.004, bottom=-0.0040),
                yz_section(0.010, 0.042, 0.013, 0.006, bottom=-0.0050),
                yz_section(0.038, 0.058, 0.014, 0.007, bottom=-0.0060),
                yz_section(0.070, 0.050, 0.011, 0.005, bottom=-0.0065),
            ]
        ),
        "lever_shell",
    )
    lever.visual(lever_shell, material=lever_metal, name="lever_shell")
    lever.visual(
        Cylinder(radius=0.0055, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    lever.visual(
        Cylinder(radius=0.0030, length=0.006),
        origin=Origin(xyz=(0.049, 0.0, -0.002)),
        material=steel,
        name="punch_pin",
    )
    lever.visual(
        Cylinder(radius=0.0022, length=0.003),
        origin=Origin(xyz=(0.049, 0.0, -0.0065)),
        material=steel,
        name="punch_tip",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.078, 0.058, 0.020)),
        mass=0.12,
        origin=Origin(xyz=(0.032, 0.0, 0.001)),
    )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tab_barrel",
    )
    tab.visual(
        Box((0.004, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=lever_metal,
        name="tab_arm",
    )
    tab.visual(
        Box((0.010, 0.010, 0.004)),
        origin=Origin(xyz=(0.003, 0.0, -0.015)),
        material=lever_metal,
        name="tab_lip",
    )
    tab.inertial = Inertial.from_geometry(
        Box((0.014, 0.012, 0.020)),
        mass=0.02,
        origin=Origin(xyz=(0.001, 0.0, -0.008)),
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(-0.039, 0.0, 0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=0.0,
            upper=1.12,
        ),
    )
    model.articulation(
        "body_to_tab",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tab,
        origin=Origin(xyz=(-0.0615, 0.0, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
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
    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    tab = object_model.get_part("tab")
    hinge = object_model.get_articulation("body_to_lever")
    tab_hinge = object_model.get_articulation("body_to_tab")

    ctx.expect_gap(
        lever,
        body,
        axis="z",
        positive_elem="lever_shell",
        negative_elem="shell",
        max_gap=0.008,
        max_penetration=0.0,
        name="lever sits just above the lower shell",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="xy",
        elem_a="lever_shell",
        elem_b="shell",
        min_overlap=0.040,
        name="lever spans the body footprint",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="xy",
        elem_a="punch_tip",
        elem_b="die_housing",
        min_overlap=0.004,
        name="punch tip stays centered over the die housing",
    )
    ctx.expect_gap(
        lever,
        body,
        axis="z",
        positive_elem="punch_tip",
        negative_elem="die_housing",
        max_gap=0.0025,
        max_penetration=0.0,
        name="punch tip hovers just above the die",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(lever, elem="lever_shell")
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_shell_aabb = ctx.part_element_world_aabb(lever, elem="lever_shell")
    ctx.check(
        "lever opens upward",
        rest_shell_aabb is not None
        and open_shell_aabb is not None
        and open_shell_aabb[1][2] > rest_shell_aabb[1][2] + 0.025,
        details=f"rest={rest_shell_aabb}, open={open_shell_aabb}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_arm")
    with ctx.pose({tab_hinge: tab_hinge.motion_limits.upper}):
        raised_tab_aabb = ctx.part_element_world_aabb(tab, elem="tab_arm")
    ctx.check(
        "locking tab folds upward",
        rest_tab_aabb is not None
        and raised_tab_aabb is not None
        and raised_tab_aabb[1][0] > rest_tab_aabb[1][0] + 0.010
        and raised_tab_aabb[1][2] > rest_tab_aabb[1][2] + 0.002,
        details=f"rest={rest_tab_aabb}, raised={raised_tab_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
