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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dutch_oven_combination_oven")

    ceramic_cream = model.material("ceramic_cream", rgba=(0.92, 0.90, 0.84, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.52, 0.50, 0.47, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.17, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.68, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")

    body_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.076, 0.000),
            (0.116, 0.018),
            (0.150, 0.058),
            (0.164, 0.118),
            (0.170, 0.154),
            (0.168, 0.162),
        ],
        [
            (0.000, 0.014),
            (0.060, 0.014),
            (0.104, 0.030),
            (0.144, 0.068),
            (0.156, 0.122),
            (0.158, 0.158),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(body_shell_geom, "body_shell"),
        material=ceramic_cream,
        name="body_shell",
    )

    base_ring_geom = CylinderGeometry(radius=0.184, height=0.028, radial_segments=64).translate(
        0.0,
        0.0,
        0.014,
    )
    body.visual(
        mesh_from_geometry(base_ring_geom, "heater_base_ring"),
        material=warm_gray,
        name="heater_base_ring",
    )

    body.visual(
        Box((0.070, 0.090, 0.030)),
        origin=Origin(xyz=(0.166, 0.0, 0.055)),
        material=warm_gray,
        name="front_control_pod",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.192, 0.0, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="control_knob",
    )

    for side_sign, side_name in ((1.0, "right"), (-1.0, "left")):
        handle_geom = tube_from_spline_points(
            [
                (-0.052, side_sign * 0.152, 0.100),
                (-0.040, side_sign * 0.182, 0.112),
                (0.040, side_sign * 0.182, 0.112),
                (0.052, side_sign * 0.152, 0.100),
            ],
            radius=0.007,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        body.visual(
            mesh_from_geometry(handle_geom, f"{side_name}_side_handle"),
            material=dark_iron,
            name=f"{side_name}_side_handle",
        )

    hinge_axis_x = -0.179
    hinge_axis_z = 0.174
    hinge_lug_y = 0.045
    hinge_lug_length = 0.034
    hinge_radius = 0.011

    for side_sign, side_name in ((1.0, "right"), (-1.0, "left")):
        body.visual(
            Cylinder(radius=hinge_radius, length=hinge_lug_length),
            origin=Origin(
                xyz=(hinge_axis_x, side_sign * hinge_lug_y, hinge_axis_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_iron,
            name=f"{side_name}_hinge_lug",
        )
        body.visual(
            Box((0.016, 0.018, 0.016)),
            origin=Origin(
                xyz=(hinge_axis_x + 0.008, side_sign * hinge_lug_y, hinge_axis_z - 0.019),
            ),
            material=dark_iron,
            name=f"{side_name}_hinge_support",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.38, 0.38, 0.19)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    lid = model.part("lid")

    lid_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.172, -0.008),
            (0.170, 0.000),
            (0.154, 0.034),
            (0.114, 0.064),
            (0.060, 0.084),
            (0.000, 0.094),
        ],
        [
            (0.154, -0.008),
            (0.150, 0.002),
            (0.136, 0.030),
            (0.102, 0.056),
            (0.052, 0.074),
            (0.000, 0.080),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    lid.visual(
        mesh_from_geometry(lid_shell_geom, "lid_shell"),
        origin=Origin(xyz=(0.179, 0.0, 0.0)),
        material=ceramic_cream,
        name="lid_shell",
    )

    lid.visual(
        Cylinder(radius=0.0105, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.018, 0.032, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.006)),
        material=dark_iron,
        name="lid_hinge_bracket",
    )

    lid_handle_geom = tube_from_spline_points(
        [
            (0.179, -0.048, 0.092),
            (0.179, -0.028, 0.114),
            (0.179, 0.028, 0.114),
            (0.179, 0.048, 0.092),
        ],
        radius=0.0065,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )
    lid.visual(
        mesh_from_geometry(lid_handle_geom, "lid_handle_arch"),
        material=dark_iron,
        name="lid_handle_arch",
    )
    for side_sign, side_name in ((1.0, "right"), (-1.0, "left")):
        lid.visual(
            Cylinder(radius=0.008, length=0.018),
            origin=Origin(xyz=(0.179, side_sign * 0.040, 0.085)),
            material=dark_iron,
            name=f"lid_handle_{side_name}_post",
        )

    vent_center_x = 0.214
    vent_center_z = 0.1005
    lid.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(vent_center_x, 0.0, 0.0955)),
        material=steel,
        name="vent_boss",
    )
    lid.visual(
        Box((0.078, 0.028, 0.003)),
        origin=Origin(xyz=(vent_center_x, 0.0, vent_center_z)),
        material=steel,
        name="vent_track_pad",
    )
    lid.visual(
        Box((0.030, 0.010, 0.002)),
        origin=Origin(xyz=(vent_center_x, 0.0, vent_center_z - 0.0005)),
        material=matte_black,
        name="vent_slot",
    )
    for side_sign, side_name in ((1.0, "right"), (-1.0, "left")):
        lid.visual(
            Box((0.058, 0.004, 0.003)),
            origin=Origin(xyz=(vent_center_x, side_sign * 0.012, vent_center_z)),
            material=dark_iron,
            name=f"vent_rail_{side_name}",
        )
    for end_sign, end_name in ((1.0, "front"), (-1.0, "rear")):
        lid.visual(
            Box((0.004, 0.024, 0.002)),
            origin=Origin(
                xyz=(vent_center_x + end_sign * 0.035, 0.0, vent_center_z),
            ),
            material=dark_iron,
            name=f"vent_stop_{end_name}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((0.37, 0.37, 0.13)),
        mass=3.2,
        origin=Origin(xyz=(0.179, 0.0, 0.040)),
    )

    vent_cover = model.part("vent_cover")
    vent_cover.visual(
        Box((0.042, 0.020, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=steel,
        name="vent_cover_plate",
    )
    vent_cover.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_iron,
        name="vent_cover_grip",
    )
    vent_cover.inertial = Inertial.from_geometry(
        Box((0.042, 0.020, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    model.articulation(
        "lid_to_vent_cover",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=vent_cover,
        origin=Origin(xyz=(vent_center_x, 0.0, vent_center_z + 0.0015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=0.0,
            upper=0.018,
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
    lid = object_model.get_part("lid")
    vent_cover = object_model.get_part("vent_cover")
    lid_hinge = object_model.get_articulation("body_to_lid")
    vent_slide = object_model.get_articulation("lid_to_vent_cover")

    with ctx.pose({lid_hinge: 0.0, vent_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="closed lid seats just above the ceramic rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.28,
            name="closed lid covers the round body opening",
        )
        ctx.expect_overlap(
            vent_cover,
            lid,
            axes="xy",
            elem_a="vent_cover_plate",
            elem_b="vent_slot",
            min_overlap=0.010,
            name="closed vent cover spans the steam vent",
        )
        ctx.expect_gap(
            vent_cover,
            lid,
            axis="z",
            positive_elem="vent_cover_plate",
            negative_elem="vent_track_pad",
            max_gap=0.004,
            max_penetration=0.0,
            name="vent cover rides on the lid track without sinking in",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: math.radians(78.0)}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05
        and open_lid_aabb[0][0] < closed_lid_aabb[0][0] - 0.02,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_vent_pos = None
    open_vent_pos = None
    with ctx.pose({lid_hinge: 0.0, vent_slide: 0.0}):
        closed_vent_pos = ctx.part_world_position(vent_cover)
    with ctx.pose({lid_hinge: 0.0, vent_slide: 0.018}):
        open_vent_pos = ctx.part_world_position(vent_cover)
        ctx.expect_within(
            vent_cover,
            lid,
            axes="y",
            inner_elem="vent_cover_plate",
            outer_elem="vent_track_pad",
            margin=0.001,
            name="vent cover stays laterally captured by the track",
        )
        ctx.expect_overlap(
            vent_cover,
            lid,
            axes="x",
            elem_a="vent_cover_plate",
            elem_b="vent_track_pad",
            min_overlap=0.030,
            name="vent cover retains fore-aft guidance at full opening",
        )
    ctx.check(
        "vent cover slides forward",
        closed_vent_pos is not None
        and open_vent_pos is not None
        and open_vent_pos[0] > closed_vent_pos[0] + 0.012,
        details=f"closed={closed_vent_pos}, open={open_vent_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
