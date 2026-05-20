from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BOSS_RADIUS = 0.016
BOSS_DEPTH = 0.008
PLATE_DEPTH = 0.008
WEB_DEPTH = 0.004
SHOULDER_LENGTH = 0.028
LINK_HEIGHT = 0.026
WEB_HEIGHT = 0.020

PRIMARY_LINK_LENGTH = 0.145
SECONDARY_LINK_LENGTH = 0.120
END_PAD_LENGTH = 0.052


def _y_cylinder_origin(x: float, y_center: float, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y_center, z), rpy=(-pi / 2.0, 0.0, 0.0))


def _add_positive_boss(part, *, x: float, name: str, material: str) -> None:
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_DEPTH),
        origin=_y_cylinder_origin(x, BOSS_DEPTH / 2.0),
        material=material,
        name=name,
    )


def _add_negative_boss(part, *, x: float, name: str, material: str) -> None:
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_DEPTH),
        origin=_y_cylinder_origin(x, -BOSS_DEPTH / 2.0),
        material=material,
        name=name,
    )


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _build_link_part(model: ArticulatedObject, name: str, length: float):
    link = model.part(name)
    _add_positive_boss(link, x=0.0, name=f"{name}_root_boss", material="link_steel")
    _add_box(
        link,
        size=(SHOULDER_LENGTH, PLATE_DEPTH, LINK_HEIGHT),
        center=(SHOULDER_LENGTH / 2.0, BOSS_DEPTH / 2.0, 0.0),
        material="link_steel",
        name=f"{name}_root_shoulder",
    )
    _add_box(
        link,
        size=(length - 2.0 * 0.024, WEB_DEPTH, WEB_HEIGHT),
        center=(length / 2.0, 0.0, 0.0),
        material="link_steel",
        name=f"{name}_web",
    )
    _add_box(
        link,
        size=(SHOULDER_LENGTH, PLATE_DEPTH, LINK_HEIGHT),
        center=(length - SHOULDER_LENGTH / 2.0, -BOSS_DEPTH / 2.0, 0.0),
        material="link_steel",
        name=f"{name}_distal_shoulder",
    )
    _add_negative_boss(link, x=length, name=f"{name}_distal_boss", material="link_steel")
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_folding_arm_chain")

    model.material("cheek_steel", rgba=(0.24, 0.26, 0.30, 1.0))
    model.material("link_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("pad_black", rgba=(0.12, 0.13, 0.14, 1.0))

    fixed_cheek = model.part("fixed_cheek")
    _add_box(
        fixed_cheek,
        size=(0.088, PLATE_DEPTH, 0.060),
        center=(-0.044, -BOSS_DEPTH / 2.0, 0.0),
        material="cheek_steel",
        name="cheek_plate",
    )
    _add_box(
        fixed_cheek,
        size=(0.022, 0.012, 0.072),
        center=(-0.072, -0.006, 0.0),
        material="cheek_steel",
        name="cheek_back_pad",
    )
    _add_box(
        fixed_cheek,
        size=(SHOULDER_LENGTH, PLATE_DEPTH, 0.036),
        center=(-SHOULDER_LENGTH / 2.0, -BOSS_DEPTH / 2.0, 0.0),
        material="cheek_steel",
        name="cheek_nose",
    )
    _add_negative_boss(fixed_cheek, x=0.0, name="cheek_root_boss", material="cheek_steel")
    fixed_cheek.inertial = Inertial.from_geometry(
        Box((0.100, 0.016, 0.074)),
        mass=0.95,
        origin=Origin(xyz=(-0.050, -0.007, 0.0)),
    )

    primary_link = _build_link_part(model, "primary_link", PRIMARY_LINK_LENGTH)
    primary_link.inertial = Inertial.from_geometry(
        Box((PRIMARY_LINK_LENGTH, 0.012, 0.030)),
        mass=0.42,
        origin=Origin(xyz=(PRIMARY_LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    secondary_link = _build_link_part(model, "secondary_link", SECONDARY_LINK_LENGTH)
    secondary_link.inertial = Inertial.from_geometry(
        Box((SECONDARY_LINK_LENGTH, 0.012, 0.030)),
        mass=0.34,
        origin=Origin(xyz=(SECONDARY_LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    end_pad = model.part("end_pad")
    _add_positive_boss(end_pad, x=0.0, name="pad_root_boss", material="pad_black")
    _add_box(
        end_pad,
        size=(SHOULDER_LENGTH, PLATE_DEPTH, 0.024),
        center=(SHOULDER_LENGTH / 2.0, BOSS_DEPTH / 2.0, 0.0),
        material="pad_black",
        name="pad_root_shoulder",
    )
    _add_box(
        end_pad,
        size=(0.040, PLATE_DEPTH, 0.026),
        center=(0.032, BOSS_DEPTH / 2.0, 0.0),
        material="pad_black",
        name="pad_body",
    )
    _add_box(
        end_pad,
        size=(0.016, PLATE_DEPTH, 0.022),
        center=(END_PAD_LENGTH - 0.008, BOSS_DEPTH / 2.0, 0.0),
        material="pad_black",
        name="pad_toe",
    )
    end_pad.inertial = Inertial.from_geometry(
        Box((END_PAD_LENGTH, 0.012, 0.028)),
        mass=0.14,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "cheek_to_primary",
        ArticulationType.REVOLUTE,
        parent=fixed_cheek,
        child=primary_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.30, effort=28.0, velocity=1.5),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.55, upper=1.55, effort=20.0, velocity=1.8),
    )
    model.articulation(
        "secondary_to_end_pad",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=end_pad,
        origin=Origin(xyz=(SECONDARY_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.95, upper=0.95, effort=10.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_cheek = object_model.get_part("fixed_cheek")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    end_pad = object_model.get_part("end_pad")

    cheek_to_primary = object_model.get_articulation("cheek_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_end_pad = object_model.get_articulation("secondary_to_end_pad")

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

    ctx.expect_contact(fixed_cheek, primary_link, name="fixed_cheek_supports_primary_link")
    ctx.expect_contact(primary_link, secondary_link, name="primary_link_supports_secondary_link")
    ctx.expect_contact(secondary_link, end_pad, name="secondary_link_supports_end_pad")

    axes_parallel = (
        tuple(cheek_to_primary.axis) == (0.0, -1.0, 0.0)
        and tuple(primary_to_secondary.axis) == (0.0, -1.0, 0.0)
        and tuple(secondary_to_end_pad.axis) == (0.0, -1.0, 0.0)
    )
    ctx.check(
        "three_parallel_revolute_axes",
        axes_parallel,
        details=(
            f"axes={cheek_to_primary.axis}, {primary_to_secondary.axis}, "
            f"{secondary_to_end_pad.axis}"
        ),
    )

    rest_pad_position = ctx.part_world_position(end_pad)
    with ctx.pose(
        cheek_to_primary=0.75,
        primary_to_secondary=0.45,
        secondary_to_end_pad=0.20,
    ):
        raised_pad_position = ctx.part_world_position(end_pad)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_raised_pose")

    pad_lifts = (
        rest_pad_position is not None
        and raised_pad_position is not None
        and raised_pad_position[2] > rest_pad_position[2] + 0.08
    )
    ctx.check(
        "positive_joint_motion_raises_end_pad",
        pad_lifts,
        details=f"rest={rest_pad_position}, raised={raised_pad_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
