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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


ALUMINUM = "finger_aluminum"
STEEL = "module_steel"
RUBBER = "pad_rubber"
Y_CYLINDER = Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _add_y_barrel(part, name: str, radius: float, length: float, xyz: tuple[float, float, float], material: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=Y_CYLINDER.rpy),
        material=material,
        name=name,
    )


def _add_box(part, name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_finger_module")

    model.material(ALUMINUM, rgba=(0.72, 0.75, 0.79, 1.0))
    model.material(STEEL, rgba=(0.24, 0.26, 0.30, 1.0))
    model.material(RUBBER, rgba=(0.08, 0.09, 0.10, 1.0))

    root_housing = model.part("root_housing")
    _add_box(root_housing, "housing_back", (0.042, 0.032, 0.034), (-0.031, 0.0, 0.0), STEEL)
    _add_box(root_housing, "housing_shoulder", (0.016, 0.022, 0.024), (-0.011, 0.0, 0.0), STEEL)
    _add_box(root_housing, "left_clevis_plate", (0.018, 0.004, 0.022), (0.0, 0.010, 0.0), STEEL)
    _add_box(root_housing, "right_clevis_plate", (0.018, 0.004, 0.022), (0.0, -0.010, 0.0), STEEL)
    _add_y_barrel(root_housing, "left_root_boss", 0.0062, 0.004, (0.0, 0.014, 0.0), STEEL)
    _add_y_barrel(root_housing, "right_root_boss", 0.0062, 0.004, (0.0, -0.014, 0.0), STEEL)
    root_housing.inertial = Inertial.from_geometry(
        Box((0.058, 0.032, 0.034)),
        mass=0.42,
        origin=Origin(xyz=(-0.023, 0.0, 0.0)),
    )

    root_link = model.part("root_link")
    _add_y_barrel(root_link, "root_knuckle_barrel", 0.0060, 0.016, (0.0, 0.0, 0.0), ALUMINUM)
    _add_box(root_link, "root_hub_block", (0.020, 0.018, 0.020), (0.015, 0.0, 0.0), ALUMINUM)
    _add_box(root_link, "root_taper_beam", (0.044, 0.014, 0.016), (0.034, 0.0, 0.0), ALUMINUM)
    _add_box(root_link, "left_middle_fork", (0.016, 0.006, 0.016), (0.060, 0.0085, 0.0), ALUMINUM)
    _add_box(root_link, "right_middle_fork", (0.016, 0.006, 0.016), (0.060, -0.0085, 0.0), ALUMINUM)
    _add_y_barrel(root_link, "left_middle_boss", 0.0052, 0.006, (0.060, 0.011, 0.0), ALUMINUM)
    _add_y_barrel(root_link, "right_middle_boss", 0.0052, 0.006, (0.060, -0.011, 0.0), ALUMINUM)
    root_link.inertial = Inertial.from_geometry(
        Box((0.076, 0.020, 0.020)),
        mass=0.16,
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
    )

    middle_link = model.part("middle_link")
    _add_y_barrel(middle_link, "middle_knuckle_barrel", 0.0052, 0.016, (0.0, 0.0, 0.0), ALUMINUM)
    _add_box(middle_link, "middle_hub_block", (0.014, 0.016, 0.018), (0.007, 0.0, 0.0), ALUMINUM)
    _add_box(middle_link, "middle_taper_beam", (0.046, 0.012, 0.014), (0.023, 0.0, 0.0), ALUMINUM)
    _add_box(middle_link, "left_distal_fork", (0.012, 0.006, 0.014), (0.046, 0.0075, 0.0), ALUMINUM)
    _add_box(middle_link, "right_distal_fork", (0.012, 0.006, 0.014), (0.046, -0.0075, 0.0), ALUMINUM)
    _add_y_barrel(middle_link, "left_distal_boss", 0.0044, 0.006, (0.046, 0.0100, 0.0), ALUMINUM)
    _add_y_barrel(middle_link, "right_distal_boss", 0.0044, 0.006, (0.046, -0.0100, 0.0), ALUMINUM)
    middle_link.inertial = Inertial.from_geometry(
        Box((0.060, 0.018, 0.018)),
        mass=0.10,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    _add_y_barrel(distal_link, "distal_knuckle_barrel", 0.0044, 0.014, (0.0, 0.0, 0.0), ALUMINUM)
    _add_box(distal_link, "distal_hub_block", (0.014, 0.014, 0.014), (0.007, 0.0, 0.0), ALUMINUM)
    _add_box(distal_link, "distal_beam", (0.022, 0.010, 0.012), (0.018, 0.0, 0.0), ALUMINUM)
    _add_box(distal_link, "pad_shoe", (0.022, 0.024, 0.006), (0.035, 0.0, -0.002), ALUMINUM)
    _add_box(distal_link, "tip_cap", (0.010, 0.018, 0.010), (0.037, 0.0, 0.002), ALUMINUM)
    distal_link.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.016)),
        mass=0.07,
        origin=Origin(xyz=(0.024, 0.0, -0.001)),
    )

    pad_tip = model.part("pad_tip")
    _add_box(pad_tip, "pad_insert", (0.022, 0.026, 0.005), (0.0, 0.0, 0.0), RUBBER)
    _add_box(pad_tip, "pad_backing", (0.008, 0.020, 0.002), (-0.006, 0.0, 0.0035), RUBBER)
    pad_tip.inertial = Inertial.from_geometry(
        Box((0.022, 0.026, 0.005)),
        mass=0.02,
        origin=Origin(),
    )

    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=root_housing,
        child=root_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.20, upper=0.95),
    )
    model.articulation(
        "middle_joint",
        ArticulationType.REVOLUTE,
        parent=root_link,
        child=middle_link,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=3.0, lower=0.00, upper=1.35),
    )
    model.articulation(
        "distal_joint",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=distal_link,
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.5, lower=0.00, upper=1.10),
    )
    model.articulation(
        "pad_mount",
        ArticulationType.FIXED,
        parent=distal_link,
        child=pad_tip,
        origin=Origin(xyz=(0.040, 0.0, -0.009)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_housing = object_model.get_part("root_housing")
    root_link = object_model.get_part("root_link")
    middle_link = object_model.get_part("middle_link")
    distal_link = object_model.get_part("distal_link")
    pad_tip = object_model.get_part("pad_tip")

    root_joint = object_model.get_articulation("root_joint")
    middle_joint = object_model.get_articulation("middle_joint")
    distal_joint = object_model.get_articulation("distal_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_finger_joints_bend_about_y_axis",
        tuple(root_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(middle_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(distal_joint.axis) == (0.0, 1.0, 0.0),
        f"axes were root={root_joint.axis}, middle={middle_joint.axis}, distal={distal_joint.axis}",
    )

    ctx.expect_contact(root_housing, root_link, contact_tol=0.0003, name="root_link_is_supported_in_root_clevis")
    ctx.expect_contact(root_link, middle_link, contact_tol=0.0003, name="middle_link_is_supported_by_root_link")
    ctx.expect_contact(middle_link, distal_link, contact_tol=0.0003, name="distal_link_is_supported_by_middle_link")
    ctx.expect_contact(distal_link, pad_tip, contact_tol=0.0002, name="pad_insert_is_mounted_to_distal_link")

    with ctx.pose({root_joint: 0.35, middle_joint: 0.80, distal_joint: 0.70}):
        ctx.expect_gap(
            middle_link,
            root_housing,
            axis="x",
            min_gap=0.03,
            name="middle_link_swings_forward_of_root_housing",
        )
        ctx.expect_gap(
            root_housing,
            distal_link,
            axis="z",
            min_gap=0.04,
            name="distal_link_curls_below_root_housing",
        )
        ctx.expect_gap(
            root_housing,
            pad_tip,
            axis="z",
            min_gap=0.06,
            name="pad_tip_stays_clear_of_root_housing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
