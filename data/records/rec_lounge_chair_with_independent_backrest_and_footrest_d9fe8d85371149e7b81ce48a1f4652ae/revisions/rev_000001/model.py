from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def rounded_pad(length: float, width: float, thickness: float, corner: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(length, width, corner, corner_segments=10),
            thickness,
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chaise_lounge")

    upholstery = model.material("upholstery", color=(0.74, 0.73, 0.70, 1.0))
    base_shell = model.material("base_shell", color=(0.36, 0.33, 0.30, 1.0))
    hinge_metal = model.material("hinge_metal", color=(0.18, 0.19, 0.20, 1.0))

    base_len = 0.84
    base_width = 0.72
    base_thickness = 0.10
    plinth_len = 0.66
    plinth_width = 0.58
    plinth_height = 0.30

    back_len = 0.76
    back_width = 0.68
    back_thickness = 0.09
    back_rest_tilt = 0.30

    leg_len = 0.46
    leg_width = 0.60
    leg_thickness = 0.09

    back_barrel_radius = 0.012
    leg_barrel_radius = 0.016
    hinge_axis_z = -(base_thickness + 0.012)
    saddle_depth = 0.032
    saddle_width = 0.52
    saddle_height = 0.055
    saddle_center_z = hinge_axis_z + 0.006

    base = model.part("base")
    base.visual(
        rounded_pad(0.84, base_width, base_thickness, 0.055, "fixed_pad"),
        origin=Origin(xyz=(0.42, 0.0, -base_thickness / 2.0)),
        material=upholstery,
        name="elem_fixed_pad",
    )
    base.visual(
        Box((plinth_len, plinth_width, plinth_height)),
        origin=Origin(xyz=(0.42, 0.0, -(base_thickness + plinth_height / 2.0))),
        material=base_shell,
        name="elem_plinth",
    )
    base.visual(
        Box((saddle_depth, saddle_width, saddle_height)),
        origin=Origin(
            xyz=(saddle_depth / 2.0, 0.0, saddle_center_z),
        ),
        material=base_shell,
        name="elem_back_hinge_saddle",
    )
    base.visual(
        Box((saddle_depth, saddle_width, saddle_height)),
        origin=Origin(
            xyz=(base_len - saddle_depth / 2.0, 0.0, saddle_center_z),
        ),
        material=base_shell,
        name="elem_leg_hinge_saddle",
    )

    backrest = model.part("backrest")
    backrest.visual(
        rounded_pad(back_len, back_width, back_thickness, 0.050, "back_panel"),
        origin=Origin(
            xyz=(-back_len / 2.0, 0.0, 0.066),
            rpy=(0.0, back_rest_tilt, 0.0),
        ),
        material=upholstery,
        name="elem_back_panel",
    )
    backrest.visual(
        Cylinder(radius=back_barrel_radius, length=0.50),
        origin=Origin(xyz=(-0.020, 0.0, -0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="elem_back_barrel",
    )
    backrest.visual(
        Box((0.072, 0.50, 0.055)),
        origin=Origin(xyz=(-0.036, 0.0, 0.006)),
        material=hinge_metal,
        name="elem_back_hinge_web",
    )

    leg_panel = model.part("leg_panel")
    leg_panel.visual(
        rounded_pad(leg_len, leg_width, leg_thickness, 0.045, "leg_panel"),
        origin=Origin(xyz=(leg_len / 2.0, 0.0, 0.066)),
        material=upholstery,
        name="elem_leg_panel",
    )
    leg_panel.visual(
        Cylinder(radius=leg_barrel_radius, length=0.52),
        origin=Origin(xyz=(leg_barrel_radius, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="elem_leg_barrel",
    )
    leg_panel.visual(
        Box((0.10, 0.50, 0.050)),
        origin=Origin(xyz=(0.050, 0.0, 0.022)),
        material=hinge_metal,
        name="elem_leg_hinge_web",
    )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "leg_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=leg_panel,
        origin=Origin(xyz=(base_len, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
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

    base = object_model.get_part("base")
    backrest = object_model.get_part("backrest")
    leg_panel = object_model.get_part("leg_panel")
    back_hinge = object_model.get_articulation("back_hinge")
    leg_hinge = object_model.get_articulation("leg_hinge")

    ctx.expect_within(
        backrest,
        base,
        axes="y",
        inner_elem="elem_back_panel",
        outer_elem="elem_fixed_pad",
        margin=0.03,
        name="backrest remains centered on the lounge width",
    )
    ctx.expect_within(
        leg_panel,
        base,
        axes="y",
        inner_elem="elem_leg_panel",
        outer_elem="elem_fixed_pad",
        margin=0.07,
        name="front leg lift remains centered on the lounge width",
    )
    ctx.expect_gap(
        base,
        backrest,
        axis="x",
        positive_elem="elem_fixed_pad",
        negative_elem="elem_back_panel",
        max_gap=0.05,
        max_penetration=0.006,
        name="backrest sits at the rear seat break",
    )
    ctx.expect_gap(
        leg_panel,
        base,
        axis="x",
        positive_elem="elem_leg_panel",
        negative_elem="elem_fixed_pad",
        max_gap=0.03,
        max_penetration=0.006,
        name="leg lift sits at the knee break",
    )
    ctx.expect_contact(
        backrest,
        base,
        elem_a="elem_back_hinge_web",
        elem_b="elem_back_hinge_saddle",
        contact_tol=1e-6,
        name="backrest hinge support touches the rear saddle block",
    )
    ctx.expect_contact(
        leg_panel,
        base,
        elem_a="elem_leg_hinge_web",
        elem_b="elem_leg_hinge_saddle",
        contact_tol=1e-6,
        name="front leg hinge support touches the front saddle block",
    )

    rest_back = ctx.part_element_world_aabb(backrest, elem="elem_back_panel")
    rest_leg = ctx.part_element_world_aabb(leg_panel, elem="elem_leg_panel")
    back_upper = back_hinge.motion_limits.upper if back_hinge.motion_limits else None
    leg_upper = leg_hinge.motion_limits.upper if leg_hinge.motion_limits else None

    with ctx.pose({back_hinge: back_upper, leg_hinge: leg_upper}):
        raised_back = ctx.part_element_world_aabb(backrest, elem="elem_back_panel")
        raised_leg = ctx.part_element_world_aabb(leg_panel, elem="elem_leg_panel")
        ctx.expect_within(
            backrest,
            base,
            axes="y",
            inner_elem="elem_back_panel",
            outer_elem="elem_fixed_pad",
            margin=0.03,
            name="raised backrest stays aligned with the seat width",
        )
        ctx.expect_within(
            leg_panel,
            base,
            axes="y",
            inner_elem="elem_leg_panel",
            outer_elem="elem_fixed_pad",
            margin=0.07,
            name="raised leg lift stays aligned with the seat width",
        )

    ctx.check(
        "backrest rotates upward from the seat break",
        bool(
            rest_back is not None
            and raised_back is not None
            and raised_back[1][2] > rest_back[1][2] + 0.16
        ),
        details=f"rest={rest_back}, raised={raised_back}",
    )
    ctx.check(
        "front leg panel lifts independently",
        bool(
            rest_leg is not None
            and raised_leg is not None
            and raised_leg[1][2] > rest_leg[1][2] + 0.10
        ),
        details=f"rest={rest_leg}, raised={raised_leg}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
