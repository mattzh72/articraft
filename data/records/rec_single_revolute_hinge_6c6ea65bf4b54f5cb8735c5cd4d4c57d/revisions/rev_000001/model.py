from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PIN_RADIUS = 0.0038
BARREL_BORE_RADIUS = 0.0042
KNUCKLE_OUTER_RADIUS = 0.0075
LEAF_THICKNESS = 0.004
SHORT_LEAF_HEIGHT = 0.100
TALL_LEAF_HEIGHT = 0.160
CENTER_KNUCKLE_HEIGHT = 0.030
KNUCKLE_END_GAP = 0.002
END_KNUCKLE_HEIGHT = (SHORT_LEAF_HEIGHT - CENTER_KNUCKLE_HEIGHT - 2 * KNUCKLE_END_GAP) / 2
HOLE_RADIUS = 0.0028

MOUNT_PROFILE = [
    (-0.055, -0.019),
    (-0.016, -0.019),
    (-0.010, -0.013),
    (-0.0045, -0.0065),
    (-0.0045, -0.0025),
    (-0.011, -0.0085),
    (-0.018, -0.015),
    (-0.055, -0.015),
]

MOVING_PROFILE = [
    (0.0045, 0.0025),
    (0.0045, 0.0065),
    (0.012, 0.0135),
    (0.018, 0.017),
    (0.070, 0.017),
    (0.070, 0.021),
    (0.019, 0.021),
    (0.011, 0.0140),
]


def extruded_profile(points: list[tuple[float, float]], height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .polyline(points)
        .close()
        .extrude(height)
        .translate((0.0, 0.0, -height / 2))
    )


def tube(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(height)
    return outer.cut(inner).translate((0.0, 0.0, z0))


def y_axis_holes(points_xz: list[tuple[float, float]], radius: float, span: float) -> cq.Workplane:
    return cq.Workplane("XZ").pushPoints(points_xz).circle(radius).extrude(span, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_hatch_hinge")

    zinc = model.material("zinc_plated_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_oxide_steel", rgba=(0.28, 0.30, 0.33, 1.0))

    mount_body = extruded_profile(MOUNT_PROFILE, SHORT_LEAF_HEIGHT).cut(
        y_axis_holes([(-0.040, -0.028), (-0.040, 0.028)], HOLE_RADIUS, 0.050)
    )
    mount_barrel_bottom = tube(
        KNUCKLE_OUTER_RADIUS,
        BARREL_BORE_RADIUS,
        END_KNUCKLE_HEIGHT,
        -SHORT_LEAF_HEIGHT / 2,
    )
    mount_barrel_top = tube(
        KNUCKLE_OUTER_RADIUS,
        BARREL_BORE_RADIUS,
        END_KNUCKLE_HEIGHT,
        SHORT_LEAF_HEIGHT / 2 - END_KNUCKLE_HEIGHT,
    )

    moving_body = extruded_profile(MOVING_PROFILE, TALL_LEAF_HEIGHT).cut(
        y_axis_holes([(0.048, -0.050), (0.048, 0.000), (0.048, 0.050)], HOLE_RADIUS, 0.050)
    )
    moving_barrel = tube(
        KNUCKLE_OUTER_RADIUS,
        BARREL_BORE_RADIUS,
        CENTER_KNUCKLE_HEIGHT,
        -CENTER_KNUCKLE_HEIGHT / 2,
    )

    mount_leaf = model.part(
        "mount_leaf",
        inertial=Inertial.from_geometry(
            Box((0.060, 0.028, SHORT_LEAF_HEIGHT)),
            mass=0.35,
            origin=Origin(xyz=(-0.025, -0.010, 0.0)),
        ),
    )
    mount_leaf.visual(
        mesh_from_cadquery(mount_body, "mount_leaf_body"),
        material=zinc,
        name="mount_plate",
    )
    mount_leaf.visual(
        mesh_from_cadquery(mount_barrel_top, "mount_barrel_top"),
        material=dark_steel,
        name="mount_barrel_top",
    )
    mount_leaf.visual(
        mesh_from_cadquery(mount_barrel_bottom, "mount_barrel_bottom"),
        material=dark_steel,
        name="mount_barrel_bottom",
    )

    moving_leaf = model.part(
        "moving_leaf",
        inertial=Inertial.from_geometry(
            Box((0.075, 0.030, TALL_LEAF_HEIGHT)),
            mass=0.45,
            origin=Origin(xyz=(0.038, 0.017, 0.0)),
        ),
    )
    moving_leaf.visual(
        mesh_from_cadquery(moving_body, "moving_leaf_body"),
        material=zinc,
        name="moving_plate",
    )
    moving_leaf.visual(
        mesh_from_cadquery(moving_barrel, "moving_barrel"),
        material=dark_steel,
        name="moving_barrel",
    )

    model.articulation(
        "mount_to_moving",
        ArticulationType.REVOLUTE,
        parent=mount_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=2.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount_leaf = object_model.get_part("mount_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("mount_to_moving")
    mount_plate = mount_leaf.get_visual("mount_plate")
    mount_barrel_top = mount_leaf.get_visual("mount_barrel_top")
    mount_barrel_bottom = mount_leaf.get_visual("mount_barrel_bottom")
    moving_plate = moving_leaf.get_visual("moving_plate")
    moving_barrel = moving_leaf.get_visual("moving_barrel")

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
        "required_parts_and_joint_present",
        all(
            item is not None
            for item in (
                mount_leaf,
                moving_leaf,
                hinge,
                mount_plate,
                mount_barrel_top,
                mount_barrel_bottom,
                moving_plate,
                moving_barrel,
            )
        ),
        "Expected both hinge leaves, the revolute articulation, and the named barrel / plate visuals.",
    )
    ctx.check(
        "hinge_axis_is_barrel_axis",
        tuple(hinge.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical barrel axis, got axis={hinge.axis!r}.",
    )
    ctx.check(
        "hinge_has_realistic_opening_range",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper >= 1.8,
        "Inspection hatch hinge should open substantially from the closed pose.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            mount_leaf,
            moving_leaf,
            axes="xy",
            min_overlap=0.010,
            elem_a=mount_barrel_top,
            elem_b=moving_barrel,
            name="moving_barrel_stays_coaxial_with_top_mount_knuckle",
        )
        ctx.expect_overlap(
            mount_leaf,
            moving_leaf,
            axes="xy",
            min_overlap=0.010,
            elem_a=mount_barrel_bottom,
            elem_b=moving_barrel,
            name="moving_barrel_stays_coaxial_with_bottom_mount_knuckle",
        )
        ctx.expect_gap(
            moving_leaf,
            mount_leaf,
            axis="z",
            min_gap=0.0015,
            max_gap=0.0035,
            positive_elem=moving_barrel,
            negative_elem=mount_barrel_bottom,
            name="center_barrel_clears_bottom_mount_knuckle",
        )
        ctx.expect_gap(
            mount_leaf,
            moving_leaf,
            axis="z",
            min_gap=0.0015,
            max_gap=0.0035,
            positive_elem=mount_barrel_top,
            negative_elem=moving_barrel,
            name="center_barrel_clears_top_mount_knuckle",
        )
        ctx.expect_gap(
            moving_leaf,
            mount_leaf,
            axis="x",
            min_gap=0.008,
            positive_elem=moving_plate,
            negative_elem=mount_plate,
            name="closed_pose_keeps_leaf_faces_on_opposite_sides_of_barrel",
        )
        ctx.expect_gap(
            moving_leaf,
            mount_leaf,
            axis="y",
            min_gap=0.004,
            max_gap=0.010,
            positive_elem=moving_plate,
            negative_elem=mount_plate,
            name="barrel_spacing_keeps_the_two_leaves_apart",
        )

        closed_box = ctx.part_element_world_aabb(moving_leaf, elem=moving_plate)

    with ctx.pose({hinge: math.pi / 2}):
        open_box = ctx.part_element_world_aabb(moving_leaf, elem=moving_plate)

    if closed_box is None or open_box is None:
        ctx.fail("moving_leaf_pose_boxes_available", "Could not resolve the moving leaf plate bounds in both test poses.")
    else:
        closed_center = tuple((lo + hi) / 2 for lo, hi in zip(closed_box[0], closed_box[1]))
        open_center = tuple((lo + hi) / 2 for lo, hi in zip(open_box[0], open_box[1]))
        ctx.check(
            "moving_leaf_sweeps_around_the_pin",
            closed_center[0] > 0.035
            and open_center[1] > 0.035
            and open_center[1] > closed_center[1] + 0.015
            and abs(open_center[0]) < closed_center[0],
            (
                "Expected the tall leaf center to move from the positive-x side of the barrel in the closed pose "
                f"to a strongly positive-y position when opened 90 degrees; got closed_center={closed_center}, "
                f"open_center={open_center}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
