from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_X = 0.19

BASE_RAIL_LENGTH = 0.58
BASE_RAIL_WIDTH = 0.045
BASE_RAIL_HEIGHT = 0.04
BASE_RAIL_Y = 0.105
BASE_RAIL_Z = -0.20

TUBE_OUTER_RADIUS = 0.085
TUBE_INNER_RADIUS = 0.077
TUBE_LENGTH = 0.46

BEARING_INNER_RADIUS = 0.0965
JOURNAL_OUTER_RADIUS = 0.095
JOURNAL_LENGTH = 0.064
COLLAR_OUTER_RADIUS = 0.110
COLLAR_INNER_RADIUS = 0.094
COLLAR_LENGTH = 0.010
COLLAR_OFFSET = 0.217

SUPPORT_OUTER_RADIUS = 0.145
SUPPORT_BORE_RADIUS = 0.124
SUPPORT_SHOULDER_RADIUS = 0.102


def x_cylinder(
    radius: float,
    length: float,
    *,
    x_center: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("YZ").cylinder(length, radius).translate((x_center, 0.0, 0.0))


def x_annulus(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    x_center: float = 0.0,
) -> cq.Workplane:
    outer = x_cylinder(outer_radius, length, x_center=x_center)
    inner = x_cylinder(inner_radius, length + 0.002, x_center=x_center)
    solid = outer.cut(inner)
    return solid


def box_at(size_x: float, size_y: float, size_z: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def rib_at(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    center: tuple[float, float, float],
    rx_deg: float = 0.0,
) -> cq.Workplane:
    rib = cq.Workplane("XY").box(size_x, size_y, size_z)
    if rx_deg:
        rib = rib.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), rx_deg)
    return rib.translate(center)


def build_base_frame() -> cq.Workplane:
    rail_left = box_at(
        BASE_RAIL_LENGTH,
        BASE_RAIL_WIDTH,
        BASE_RAIL_HEIGHT,
        center=(0.0, BASE_RAIL_Y, BASE_RAIL_Z),
    )
    rail_right = box_at(
        BASE_RAIL_LENGTH,
        BASE_RAIL_WIDTH,
        BASE_RAIL_HEIGHT,
        center=(0.0, -BASE_RAIL_Y, BASE_RAIL_Z),
    )
    front_tie = box_at(0.05, 0.18, 0.024, center=(-SUPPORT_X, 0.0, -0.198))
    center_tie = box_at(0.05, 0.18, 0.024, center=(0.0, 0.0, -0.198))
    rear_tie = box_at(0.05, 0.18, 0.024, center=(SUPPORT_X, 0.0, -0.198))
    return rail_left.union(rail_right).union(front_tie).union(center_tie).union(rear_tie)


def build_support_housing() -> cq.Workplane:
    ring_shell = x_annulus(SUPPORT_OUTER_RADIUS, SUPPORT_BORE_RADIUS, 0.034)
    inner_shoulder = x_annulus(
        SUPPORT_BORE_RADIUS,
        SUPPORT_SHOULDER_RADIUS,
        0.008,
        x_center=0.013,
    )

    top_band = box_at(0.040, 0.168, 0.026, center=(0.0, 0.0, 0.132))
    lower_bridge = box_at(0.044, 0.118, 0.022, center=(0.0, 0.0, -0.166))

    right_leg = box_at(0.040, 0.028, 0.114, center=(0.0, 0.105, -0.103))
    left_leg = box_at(0.040, 0.028, 0.114, center=(0.0, -0.105, -0.103))

    right_foot = box_at(0.052, 0.062, 0.028, center=(0.0, BASE_RAIL_Y, -0.166))
    left_foot = box_at(0.052, 0.062, 0.028, center=(0.0, -BASE_RAIL_Y, -0.166))

    right_outer_rib = rib_at(
        0.024,
        0.024,
        0.082,
        center=(0.0, 0.118, 0.070),
        rx_deg=12.0,
    )
    left_outer_rib = rib_at(
        0.024,
        0.024,
        0.082,
        center=(0.0, -0.118, 0.070),
        rx_deg=-12.0,
    )

    return (
        ring_shell.union(inner_shoulder)
        .union(top_band)
        .union(lower_bridge)
        .union(right_leg)
        .union(left_leg)
        .union(right_foot)
        .union(left_foot)
        .union(right_outer_rib)
        .union(left_outer_rib)
    )


def build_support_bearing_ring() -> cq.Workplane:
    return x_annulus(0.124, BEARING_INNER_RADIUS, 0.022, x_center=0.003)


def build_support_retainer_cap() -> cq.Workplane:
    inner_lip = x_annulus(0.1245, 0.100, 0.010, x_center=-0.013)
    outer_rim = x_annulus(SUPPORT_OUTER_RADIUS, 0.123, 0.006, x_center=-0.019)
    return inner_lip.union(outer_rim)


def build_carrier_tube() -> cq.Workplane:
    return x_annulus(TUBE_OUTER_RADIUS, TUBE_INNER_RADIUS, TUBE_LENGTH)


def build_carrier_journal(x_center: float) -> cq.Workplane:
    return x_annulus(
        JOURNAL_OUTER_RADIUS,
        TUBE_INNER_RADIUS,
        JOURNAL_LENGTH,
        x_center=x_center,
    )


def build_carrier_collar(x_center: float) -> cq.Workplane:
    return x_annulus(
        COLLAR_OUTER_RADIUS,
        COLLAR_INNER_RADIUS,
        COLLAR_LENGTH,
        x_center=x_center,
    )


def build_carrier_bracket() -> cq.Workplane:
    saddle_block = cq.Workplane("XY").box(0.080, 0.042, 0.020).translate((0.0, 0.0, 0.091))
    upright = cq.Workplane("XY").box(0.060, 0.010, 0.050).translate((0.0, 0.016, 0.123))
    top_flange = cq.Workplane("XY").box(0.032, 0.028, 0.010).translate((0.0, 0.010, 0.148))
    gusset_left = rib_at(
        0.060,
        0.006,
        0.040,
        center=(0.0, 0.009, 0.113),
        rx_deg=-34.0,
    )
    gusset_right = rib_at(
        0.060,
        0.006,
        0.040,
        center=(0.0, -0.009, 0.113),
        rx_deg=34.0,
    )
    saddle_cut = x_cylinder(TUBE_OUTER_RADIUS, 0.100)

    return saddle_block.union(upright).union(top_flange).union(gusset_left).union(gusset_right).cut(saddle_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tube_rotator_module")

    base_mat = model.material("base_coat", rgba=(0.16, 0.17, 0.19, 1.0))
    support_mat = model.material("support_cast", rgba=(0.58, 0.60, 0.63, 1.0))
    machined_mat = model.material("machined_steel", rgba=(0.78, 0.79, 0.80, 1.0))
    carrier_mat = model.material("carrier_satin", rgba=(0.34, 0.37, 0.41, 1.0))
    bracket_mat = model.material("bracket_anodized", rgba=(0.20, 0.22, 0.25, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(build_base_frame(), "tube_rotator_base"),
        name="frame",
        material=base_mat,
    )

    front_support = model.part("front_support")
    front_support.visual(
        mesh_from_cadquery(build_support_housing(), "front_support_housing"),
        name="housing",
        material=support_mat,
    )
    front_support.visual(
        mesh_from_cadquery(build_support_bearing_ring(), "front_support_bearing_ring"),
        name="bearing_ring",
        material=machined_mat,
    )
    front_support.visual(
        mesh_from_cadquery(build_support_retainer_cap(), "front_support_retainer_cap"),
        name="retainer_cap",
        material=machined_mat,
    )

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(build_support_housing(), "rear_support_housing"),
        name="housing",
        material=support_mat,
    )
    rear_support.visual(
        mesh_from_cadquery(build_support_bearing_ring(), "rear_support_bearing_ring"),
        name="bearing_ring",
        material=machined_mat,
    )
    rear_support.visual(
        mesh_from_cadquery(build_support_retainer_cap(), "rear_support_retainer_cap"),
        name="retainer_cap",
        material=machined_mat,
    )

    carrier = model.part("carrier")
    carrier.visual(
        mesh_from_cadquery(build_carrier_tube(), "carrier_tube_shell"),
        name="tube_shell",
        material=carrier_mat,
    )
    carrier.visual(
        mesh_from_cadquery(build_carrier_journal(-SUPPORT_X), "carrier_front_journal"),
        name="front_journal",
        material=machined_mat,
    )
    carrier.visual(
        mesh_from_cadquery(build_carrier_journal(SUPPORT_X), "carrier_rear_journal"),
        name="rear_journal",
        material=machined_mat,
    )
    carrier.visual(
        mesh_from_cadquery(build_carrier_collar(-COLLAR_OFFSET), "carrier_front_collar"),
        name="front_collar",
        material=machined_mat,
    )
    carrier.visual(
        mesh_from_cadquery(build_carrier_collar(COLLAR_OFFSET), "carrier_rear_collar"),
        name="rear_collar",
        material=machined_mat,
    )

    bracket = model.part("carrier_bracket")
    bracket.visual(
        mesh_from_cadquery(build_carrier_bracket(), "carrier_mount_bracket"),
        name="mount_bracket",
        material=bracket_mat,
    )

    model.articulation(
        "base_to_front_support",
        ArticulationType.FIXED,
        parent=base,
        child=front_support,
        origin=Origin(xyz=(-SUPPORT_X, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_rear_support",
        ArticulationType.FIXED,
        parent=base,
        child=rear_support,
        origin=Origin(xyz=(SUPPORT_X, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "carrier_roll",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=3.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "carrier_to_bracket",
        ArticulationType.FIXED,
        parent=carrier,
        child=bracket,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    front_support = object_model.get_part("front_support")
    rear_support = object_model.get_part("rear_support")
    carrier = object_model.get_part("carrier")
    bracket = object_model.get_part("carrier_bracket")
    roll = object_model.get_articulation("carrier_roll")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_isolated_part(
        carrier,
        reason=(
            "The carrier is intentionally centered with running clearance inside opposed bearing hoops; "
            "its support path is proven by bearing containment and axial capture checks rather than static contact."
        ),
    )
    ctx.allow_isolated_part(
        bracket,
        reason="The bracket is rigidly fixed to the intentionally clearance-supported rotating carrier.",
    )

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

    ctx.expect_contact(front_support, base, name="front support is grounded on the frame")
    ctx.expect_contact(rear_support, base, name="rear support is grounded on the frame")
    ctx.expect_contact(bracket, carrier, name="carrier bracket is fixed onto the rotating tube")
    ctx.expect_origin_distance(
        front_support,
        rear_support,
        axes="x",
        min_dist=0.37,
        max_dist=0.39,
        name="support hoops stay balanced about the carrier centerline",
    )

    with ctx.pose({roll: 0.0}):
        ctx.expect_within(
            carrier,
            front_support,
            axes="yz",
            inner_elem="front_journal",
            outer_elem="bearing_ring",
            name="front journal stays inside the front bearing hoop footprint",
        )
        ctx.expect_within(
            carrier,
            rear_support,
            axes="yz",
            inner_elem="rear_journal",
            outer_elem="bearing_ring",
            name="rear journal stays inside the rear bearing hoop footprint",
        )
        ctx.expect_gap(
            front_support,
            carrier,
            axis="x",
            positive_elem="retainer_cap",
            negative_elem="front_collar",
            max_penetration=1e-6,
            max_gap=0.0005,
            name="front retainer cap captures the carrier collar with minimal endplay",
        )
        ctx.expect_gap(
            carrier,
            rear_support,
            axis="x",
            positive_elem="rear_collar",
            negative_elem="retainer_cap",
            max_penetration=1e-6,
            max_gap=0.0005,
            name="rear retainer cap captures the carrier collar with minimal endplay",
        )
        ctx.expect_within(
            carrier,
            front_support,
            axes="yz",
            inner_elem="front_collar",
            outer_elem="retainer_cap",
            name="front collar stays projected inside the front retainer cap",
        )
        ctx.expect_within(
            carrier,
            rear_support,
            axes="yz",
            inner_elem="rear_collar",
            outer_elem="retainer_cap",
            name="rear collar stays projected inside the rear retainer cap",
        )
        ctx.expect_gap(
            bracket,
            front_support,
            axis="x",
            min_gap=0.12,
            name="bracket stays between the two support hoops on the front side",
        )
        ctx.expect_gap(
            rear_support,
            bracket,
            axis="x",
            min_gap=0.12,
            name="bracket stays between the two support hoops on the rear side",
        )

    with ctx.pose({roll: math.pi}):
        ctx.expect_gap(
            bracket,
            base,
            axis="z",
            min_gap=0.02,
            name="inverted bracket still clears the base rails",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_fixed=True,
        name="rolling carrier clears supports and frame throughout its motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
