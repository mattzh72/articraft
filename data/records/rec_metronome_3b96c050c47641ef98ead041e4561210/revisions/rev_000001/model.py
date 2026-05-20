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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_metronome")

    wood = model.material("walnut_wood", rgba=(0.36, 0.22, 0.13, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.11, 0.09, 0.08, 1.0))

    base_width = 0.26
    base_depth = 0.18
    plinth_height = 0.022
    shell_height = 0.34
    top_width = 0.12
    top_depth = 0.08
    panel_thickness = 0.010

    side_angle = math.atan(((base_width - top_width) * 0.5) / shell_height)
    front_angle = math.atan(((base_depth - top_depth) * 0.5) / shell_height)
    shell_mid_z = plinth_height + shell_height * 0.5 - 0.004

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((base_width, base_depth, plinth_height + shell_height)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, (plinth_height + shell_height) * 0.5)),
    )

    housing.visual(
        Box((base_width, base_depth, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=wood,
        name="plinth",
    )
    housing.visual(
        Box((panel_thickness, base_depth, shell_height)),
        origin=Origin(
            xyz=(-(base_width + top_width) * 0.25, 0.0, shell_mid_z),
            rpy=(0.0, side_angle, 0.0),
        ),
        material=wood,
        name="left_shell",
    )
    housing.visual(
        Box((panel_thickness, base_depth, shell_height)),
        origin=Origin(
            xyz=((base_width + top_width) * 0.25, 0.0, shell_mid_z),
            rpy=(0.0, -side_angle, 0.0),
        ),
        material=wood,
        name="right_shell",
    )
    housing.visual(
        Box((base_width - 0.02, panel_thickness, shell_height)),
        origin=Origin(
            xyz=(0.0, (base_depth + top_depth) * 0.25, shell_mid_z),
            rpy=(-front_angle, 0.0, 0.0),
        ),
        material=wood,
        name="rear_shell",
    )
    housing.visual(
        Box((0.18, panel_thickness, 0.072)),
        origin=Origin(
            xyz=(0.0, -(base_depth + top_depth) * 0.25, plinth_height + 0.036),
            rpy=(front_angle, 0.0, 0.0),
        ),
        material=wood,
        name="front_apron",
    )
    housing.visual(
        Box((0.032, panel_thickness, 0.245)),
        origin=Origin(
            xyz=(-0.066, -0.058, plinth_height + 0.180),
            rpy=(front_angle, 0.0, 0.0),
        ),
        material=wood,
        name="left_stile",
    )
    housing.visual(
        Box((0.032, panel_thickness, 0.245)),
        origin=Origin(
            xyz=(0.066, -0.058, plinth_height + 0.180),
            rpy=(front_angle, 0.0, 0.0),
        ),
        material=wood,
        name="right_stile",
    )
    housing.visual(
        Box((0.126, panel_thickness, 0.046)),
        origin=Origin(
            xyz=(0.0, -0.046, plinth_height + 0.311),
            rpy=(front_angle, 0.0, 0.0),
        ),
        material=wood,
        name="top_bridge",
    )
    housing.visual(
        Box((0.055, top_depth + 0.008, 0.014)),
        origin=Origin(xyz=(-0.034, 0.0, plinth_height + shell_height - 0.007)),
        material=wood,
        name="top_left_cap",
    )
    housing.visual(
        Box((0.055, top_depth + 0.008, 0.014)),
        origin=Origin(xyz=(0.034, 0.0, plinth_height + shell_height - 0.007)),
        material=wood,
        name="top_right_cap",
    )
    housing.visual(
        Box((0.030, 0.020, 0.235)),
        origin=Origin(xyz=(0.0, 0.054, plinth_height + 0.182)),
        material=dark_panel,
        name="scale_backer",
    )
    housing.visual(
        Box((0.010, 0.032, 0.028)),
        origin=Origin(xyz=(-0.012, 0.0, plinth_height + shell_height - 0.028)),
        material=dark_panel,
        name="left_pivot_cheek",
    )
    housing.visual(
        Box((0.010, 0.032, 0.028)),
        origin=Origin(xyz=(0.012, 0.0, plinth_height + shell_height - 0.028)),
        material=dark_panel,
        name="right_pivot_cheek",
    )
    housing.visual(
        Box((0.024, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.019, plinth_height + shell_height - 0.028)),
        material=dark_panel,
        name="front_pivot_support",
    )
    housing.visual(
        Box((0.024, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.019, plinth_height + shell_height - 0.028)),
        material=dark_panel,
        name="rear_pivot_support",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.003, length=0.37),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material="dark_panel",
        name="rod_shaft",
    )
    pendulum.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material="dark_panel",
        name="pivot_barrel",
    )
    pendulum.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material="dark_panel",
        name="upper_tip",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.028, 0.028, 0.39)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    weight = model.part("weight")
    weight.visual(
        Cylinder(radius=0.016, length=0.096),
        origin=Origin(xyz=(0.0, -0.025, -0.17), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="dark_panel",
        name="bob_mass",
    )
    weight.visual(
        Box((0.030, 0.010, 0.034)),
        origin=Origin(xyz=(0.0, -0.008, -0.17)),
        material="dark_panel",
        name="front_bridge",
    )
    weight.visual(
        Box((0.008, 0.010, 0.036)),
        origin=Origin(xyz=(-0.009, -0.003, -0.17)),
        material="dark_panel",
        name="left_clamp",
    )
    weight.visual(
        Box((0.008, 0.010, 0.036)),
        origin=Origin(xyz=(0.009, -0.003, -0.17)),
        material="dark_panel",
        name="right_clamp",
    )
    weight.inertial = Inertial.from_geometry(
        Box((0.10, 0.05, 0.05)),
        mass=0.28,
        origin=Origin(xyz=(0.0, -0.018, -0.17)),
    )

    brass = model.material("brass", rgba=(0.74, 0.60, 0.27, 1.0))
    key_local_y = 0.020
    key_local_z = -0.018
    escutcheon_length = 0.003
    escutcheon_center_offset = panel_thickness * 0.5 + escutcheon_length * 0.5
    key_axis_origin_offset = panel_thickness * 0.5 + escutcheon_length

    housing.visual(
        Cylinder(radius=0.022, length=escutcheon_length),
        origin=Origin(
            xyz=(
                (base_width + top_width) * 0.25
                + math.cos(side_angle) * escutcheon_center_offset
                - math.sin(side_angle) * key_local_z,
                key_local_y,
                shell_mid_z
                + math.sin(side_angle) * escutcheon_center_offset
                + math.cos(side_angle) * key_local_z,
            ),
            rpy=(0.0, math.pi * 0.5 - side_angle, 0.0),
        ),
        material=brass,
        name="key_escutcheon",
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="key_spindle",
    )
    key.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="key_hub",
    )
    key.visual(
        Box((0.008, 0.038, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=brass,
        name="key_wing",
    )
    key.inertial = Inertial.from_geometry(
        Box((0.04, 0.02, 0.016)),
        mass=0.05,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    key_mount = Origin(
        xyz=(
            (base_width + top_width) * 0.25
            + math.cos(side_angle) * key_axis_origin_offset
            - math.sin(side_angle) * key_local_z,
            key_local_y,
            shell_mid_z
            + math.sin(side_angle) * key_axis_origin_offset
            + math.cos(side_angle) * key_local_z,
        ),
        rpy=(0.0, -side_angle, 0.0),
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.0, plinth_height + shell_height - 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=1.6,
            lower=-0.40,
            upper=0.40,
        ),
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.12,
            lower=-0.055,
            upper=0.055,
        ),
    )
    model.articulation(
        "housing_to_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=key_mount,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
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

    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    key = object_model.get_part("winding_key")

    swing_joint = object_model.get_articulation("housing_to_pendulum")
    slide_joint = object_model.get_articulation("pendulum_to_weight")
    key_joint = object_model.get_articulation("housing_to_key")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    housing_aabb = ctx.part_world_aabb(housing)
    tip_aabb = ctx.part_element_world_aabb(pendulum, elem="upper_tip")
    ctx.check(
        "pendulum rod protrudes above the housing",
        housing_aabb is not None
        and tip_aabb is not None
        and tip_aabb[1][2] > housing_aabb[1][2] + 0.03,
        details=f"housing_aabb={housing_aabb}, tip_aabb={tip_aabb}",
    )

    ctx.expect_contact(
        key,
        housing,
        elem_a="key_spindle",
        elem_b="key_escutcheon",
        contact_tol=0.0015,
        name="winding key sits flush on the right side panel",
    )

    bob_rest = aabb_center(ctx.part_element_world_aabb(weight, elem="bob_mass"))
    with ctx.pose({swing_joint: swing_joint.motion_limits.upper}):
        bob_swung = aabb_center(ctx.part_element_world_aabb(weight, elem="bob_mass"))
        ctx.expect_contact(
            key,
            housing,
            elem_a="key_spindle",
            elem_b="key_escutcheon",
            contact_tol=0.0015,
            name="key remains seated while rotating",
        )
    ctx.check(
        "pendulum swings laterally",
        bob_rest is not None and bob_swung is not None and abs(bob_swung[0] - bob_rest[0]) > 0.05,
        details=f"bob_rest={bob_rest}, bob_swung={bob_swung}",
    )

    with ctx.pose({slide_joint: slide_joint.motion_limits.lower}):
        bob_low = aabb_center(ctx.part_element_world_aabb(weight, elem="bob_mass"))
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        bob_high = aabb_center(ctx.part_element_world_aabb(weight, elem="bob_mass"))
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="z",
            elem_a="front_bridge",
            elem_b="rod_shaft",
            min_overlap=0.02,
            name="raised weight remains aligned with the rod",
        )
    ctx.check(
        "weight slides upward along the pendulum rod",
        bob_low is not None and bob_high is not None and bob_high[2] > bob_low[2] + 0.08,
        details=f"bob_low={bob_low}, bob_high={bob_high}",
    )

    ctx.check(
        "articulation types match the mechanism",
        swing_joint.articulation_type == ArticulationType.REVOLUTE
        and slide_joint.articulation_type == ArticulationType.PRISMATIC
        and key_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"swing={swing_joint.articulation_type}, "
            f"slide={slide_joint.articulation_type}, "
            f"key={key_joint.articulation_type}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
