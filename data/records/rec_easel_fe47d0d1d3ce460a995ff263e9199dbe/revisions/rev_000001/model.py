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
    tube_from_spline_points,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_sign_display_easel")

    chrome = model.material("chrome", rgba=(0.83, 0.85, 0.88, 1.0))
    poster_white = model.material("poster_white", rgba=(0.96, 0.96, 0.94, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    frame_width = 0.56
    frame_height = 0.88
    frame_tube_radius = 0.015
    frame_half_x = frame_width * 0.5 - frame_tube_radius
    frame_bottom_z = 0.025
    frame_top_z = frame_bottom_z + frame_height - 2.0 * frame_tube_radius
    frame_center_z = 0.5 * (frame_bottom_z + frame_top_z)

    frame = model.part("frame")
    frame_loop = wire_from_points(
        [
            (-frame_half_x, 0.0, frame_bottom_z),
            (-frame_half_x, 0.0, frame_top_z),
            (frame_half_x, 0.0, frame_top_z),
            (frame_half_x, 0.0, frame_bottom_z),
        ],
        radius=frame_tube_radius,
        radial_segments=18,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.035,
        corner_segments=10,
    )
    frame.visual(
        mesh_from_geometry(frame_loop, "sign_easel_frame_loop"),
        material=chrome,
        name="frame_loop",
    )
    frame.visual(
        Box((0.50, 0.008, 0.81)),
        origin=Origin(xyz=(0.0, 0.0, frame_center_z)),
        material=poster_white,
        name="sign_panel",
    )
    frame.visual(
        Box((0.050, 0.025648, 0.060)),
        origin=Origin(xyz=(-0.255, 0.002824, 0.835)),
        material=chrome,
        name="left_hinge_mount",
    )
    frame.visual(
        Box((0.050, 0.025648, 0.060)),
        origin=Origin(xyz=(0.255, 0.002824, 0.835)),
        material=chrome,
        name="right_hinge_mount",
    )
    frame.visual(
        Box((0.110, 0.025892, 0.085)),
        origin=Origin(xyz=(0.0, -0.002946, 0.480)),
        material=chrome,
        name="rear_hinge_mount",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.08, 0.90)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, frame_center_z)),
    )

    left_front_leg = model.part("left_front_leg")
    left_leg_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.055, -0.140),
            (0.0, 0.185, -0.824),
        ],
        radius=0.009,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    left_front_leg.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="left_hinge_barrel",
    )
    left_front_leg.visual(
        mesh_from_geometry(left_leg_geom, "left_front_leg_tube"),
        material=chrome,
        name="left_front_leg_strut",
    )
    left_front_leg.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(
            xyz=(0.0, 0.185, -0.824),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="left_front_foot",
    )
    left_front_leg.inertial = Inertial.from_geometry(
        Box((0.05, 0.24, 0.84)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.095, -0.410)),
    )

    right_front_leg = model.part("right_front_leg")
    right_leg_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.055, -0.140),
            (0.0, 0.185, -0.824),
        ],
        radius=0.009,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    right_front_leg.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="right_hinge_barrel",
    )
    right_front_leg.visual(
        mesh_from_geometry(right_leg_geom, "right_front_leg_tube"),
        material=chrome,
        name="right_front_leg_strut",
    )
    right_front_leg.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(
            xyz=(0.0, 0.185, -0.824),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="right_front_foot",
    )
    right_front_leg.inertial = Inertial.from_geometry(
        Box((0.05, 0.24, 0.84)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.095, -0.410)),
    )

    rear_kickstand = model.part("rear_kickstand")
    rear_stand_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, -0.085, -0.120),
            (0.0, -0.220, -0.472),
        ],
        radius=0.010,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    rear_kickstand.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="rear_hinge_barrel",
    )
    rear_kickstand.visual(
        mesh_from_geometry(rear_stand_geom, "rear_kickstand_strut"),
        material=chrome,
        name="rear_kickstand_strut",
    )
    rear_kickstand.visual(
        Cylinder(radius=0.008, length=0.140),
        origin=Origin(
            xyz=(0.0, -0.220, -0.472),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="rear_foot",
    )
    rear_kickstand.inertial = Inertial.from_geometry(
        Box((0.16, 0.26, 0.50)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.110, -0.236)),
    )

    model.articulation(
        "frame_to_left_front_leg",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_front_leg,
        origin=Origin(xyz=(-0.255, 0.024, 0.835)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.25,
            upper=0.18,
        ),
    )
    model.articulation(
        "frame_to_right_front_leg",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_front_leg,
        origin=Origin(xyz=(0.255, 0.024, 0.835)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.25,
            upper=0.18,
        ),
    )
    model.articulation(
        "frame_to_rear_kickstand",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_kickstand,
        origin=Origin(xyz=(0.0, -0.024, 0.480)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.1,
            lower=-0.25,
            upper=0.35,
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

    frame = object_model.get_part("frame")
    left_front_leg = object_model.get_part("left_front_leg")
    right_front_leg = object_model.get_part("right_front_leg")
    rear_kickstand = object_model.get_part("rear_kickstand")
    left_hinge = object_model.get_articulation("frame_to_left_front_leg")
    right_hinge = object_model.get_articulation("frame_to_right_front_leg")
    rear_hinge = object_model.get_articulation("frame_to_rear_kickstand")

    ctx.expect_gap(
        left_front_leg,
        frame,
        axis="y",
        positive_elem="left_front_foot",
        negative_elem="sign_panel",
        min_gap=0.15,
        name="left front foot stands ahead of frame",
    )
    ctx.expect_gap(
        right_front_leg,
        frame,
        axis="y",
        positive_elem="right_front_foot",
        negative_elem="sign_panel",
        min_gap=0.15,
        name="right front foot stands ahead of frame",
    )
    ctx.expect_gap(
        frame,
        rear_kickstand,
        axis="y",
        positive_elem="sign_panel",
        negative_elem="rear_foot",
        min_gap=0.18,
        name="rear kickstand foot stands behind frame",
    )

    open_left_foot = ctx.part_element_world_aabb(left_front_leg, elem="left_front_foot")
    open_right_foot = ctx.part_element_world_aabb(right_front_leg, elem="right_front_foot")
    open_rear_foot = ctx.part_element_world_aabb(rear_kickstand, elem="rear_foot")
    with ctx.pose(
        {
            left_hinge: -0.18,
            right_hinge: -0.18,
            rear_hinge: 0.28,
        }
    ):
        folded_left_foot = ctx.part_element_world_aabb(left_front_leg, elem="left_front_foot")
        folded_right_foot = ctx.part_element_world_aabb(right_front_leg, elem="right_front_foot")
        folded_rear_foot = ctx.part_element_world_aabb(rear_kickstand, elem="rear_foot")

    def elem_center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "front legs fold back toward frame",
        (
            open_left_foot is not None
            and open_right_foot is not None
            and folded_left_foot is not None
            and folded_right_foot is not None
            and elem_center_y(folded_left_foot) < elem_center_y(open_left_foot) - 0.12
            and elem_center_y(folded_right_foot) < elem_center_y(open_right_foot) - 0.12
        ),
        details=(
            f"open_left_y={elem_center_y(open_left_foot)}, "
            f"folded_left_y={elem_center_y(folded_left_foot)}, "
            f"open_right_y={elem_center_y(open_right_foot)}, "
            f"folded_right_y={elem_center_y(folded_right_foot)}"
        ),
    )
    ctx.check(
        "rear kickstand folds toward frame",
        (
            open_rear_foot is not None
            and folded_rear_foot is not None
            and elem_center_y(folded_rear_foot) > elem_center_y(open_rear_foot) + 0.10
        ),
        details=(
            f"open_rear_y={elem_center_y(open_rear_foot)}, "
            f"folded_rear_y={elem_center_y(folded_rear_foot)}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
