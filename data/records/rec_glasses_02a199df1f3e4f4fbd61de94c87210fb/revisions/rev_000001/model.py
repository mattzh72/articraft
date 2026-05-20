from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _managed_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_front_frame_mesh():
    frame_depth = 0.0042
    rim_outer_w = 0.050
    rim_outer_h = 0.032
    rim_inner_w = 0.038
    rim_inner_h = 0.022
    rim_center_y = 0.033

    left_rim = ExtrudeWithHolesGeometry(
        rounded_rect_profile(rim_outer_h, rim_outer_w, 0.0080),
        [rounded_rect_profile(rim_inner_h, rim_inner_w, 0.0058)],
        height=frame_depth,
        center=True,
    ).rotate_y(pi / 2.0)
    left_rim.translate(0.0, rim_center_y, 0.0)

    right_rim = ExtrudeWithHolesGeometry(
        rounded_rect_profile(rim_outer_h, rim_outer_w, 0.0080),
        [rounded_rect_profile(rim_inner_h, rim_inner_w, 0.0058)],
        height=frame_depth,
        center=True,
    ).rotate_y(pi / 2.0)
    right_rim.translate(0.0, -rim_center_y, 0.0)

    bridge = BoxGeometry((frame_depth, 0.018, 0.006)).translate(0.0, 0.0, -0.002)
    brow = BoxGeometry((frame_depth, 0.072, 0.004)).translate(0.0, 0.0, 0.012)
    left_hinge_pad = BoxGeometry((0.003, 0.0048, 0.010)).translate(-0.0015, 0.057, 0.006)
    right_hinge_pad = BoxGeometry((0.003, 0.0048, 0.010)).translate(-0.0015, -0.057, 0.006)

    frame_geom = left_rim
    frame_geom = frame_geom.merge(right_rim)
    frame_geom = frame_geom.merge(bridge)
    frame_geom = frame_geom.merge(brow)
    frame_geom = frame_geom.merge(left_hinge_pad)
    frame_geom = frame_geom.merge(right_hinge_pad)
    return _managed_mesh("travel_glasses_front_frame", frame_geom)


def _build_temple_sleeve_mesh():
    sleeve = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.0080, 0.0048, 0.0018),
        [rounded_rect_profile(0.0060, 0.0032, 0.0011)],
        height=0.046,
        center=True,
    ).rotate_y(pi / 2.0)
    sleeve.translate(-0.029, 0.0, 0.0)

    hinge_leaf = BoxGeometry((0.013, 0.0054, 0.0088)).translate(-0.0015, 0.0, 0.001)
    hinge_barrel = CylinderGeometry(0.0018, 0.010, radial_segments=18).translate(0.0, 0.0, 0.006)

    sleeve_geom = hinge_leaf
    sleeve_geom = sleeve_geom.merge(sleeve)
    sleeve_geom = sleeve_geom.merge(hinge_barrel)
    return _managed_mesh("travel_glasses_temple_sleeve", sleeve_geom)


def _build_temple_rear_mesh():
    return _managed_mesh(
        "travel_glasses_temple_rear",
        sweep_profile_along_spline(
            [
                (-0.024, 0.0, 0.0),
                (-0.070, 0.0, -0.004),
                (-0.098, 0.0, -0.014),
                (-0.116, 0.0, -0.030),
            ],
            profile=rounded_rect_profile(0.0046, 0.0026, 0.0008),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
    )


def _build_lens_mesh():
    return _managed_mesh(
        "travel_glasses_lens",
        ExtrudeGeometry(
            rounded_rect_profile(0.0205, 0.0360, 0.0048),
            0.0012,
            center=True,
        ).rotate_y(pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_travel_glasses")

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.11, 0.10, 0.10, 1.0))
    lens_tint = model.material("lens_tint", rgba=(0.72, 0.80, 0.86, 0.35))

    frame_mesh = _build_front_frame_mesh()
    sleeve_mesh = _build_temple_sleeve_mesh()
    rear_mesh = _build_temple_rear_mesh()
    lens_mesh = _build_lens_mesh()

    frame = model.part("frame")
    frame.visual(
        frame_mesh,
        material=graphite,
        name="front_frame",
    )

    left_lens = model.part("left_lens")
    left_lens.visual(
        lens_mesh,
        material=lens_tint,
        name="lens",
    )

    right_lens = model.part("right_lens")
    right_lens.visual(
        lens_mesh,
        material=lens_tint,
        name="lens",
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        sleeve_mesh,
        material=satin_black,
        name="sleeve_shell",
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        sleeve_mesh,
        material=satin_black,
        name="sleeve_shell",
    )

    left_tip = model.part("left_tip")
    left_tip.visual(
        Box((0.048, 0.0018, 0.0042)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=graphite,
        name="insert_blade",
    )
    left_tip.visual(
        Box((0.018, 0.0032, 0.0016)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=graphite,
        name="guide_shoe",
    )
    left_tip.visual(
        rear_mesh,
        material=dark_rubber,
        name="rear_hook",
    )

    right_tip = model.part("right_tip")
    right_tip.visual(
        Box((0.048, 0.0018, 0.0042)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=graphite,
        name="insert_blade",
    )
    right_tip.visual(
        Box((0.018, 0.0032, 0.0016)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=graphite,
        name="guide_shoe",
    )
    right_tip.visual(
        rear_mesh,
        material=dark_rubber,
        name="rear_hook",
    )

    model.articulation(
        "frame_to_left_lens",
        ArticulationType.FIXED,
        parent=frame,
        child=left_lens,
        origin=Origin(xyz=(0.0, 0.033, 0.0)),
    )
    model.articulation(
        "frame_to_right_lens",
        ArticulationType.FIXED,
        parent=frame,
        child=right_lens,
        origin=Origin(xyz=(0.0, -0.033, 0.0)),
    )
    model.articulation(
        "frame_to_left_temple",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_temple,
        origin=Origin(xyz=(-0.0080, 0.0621, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "frame_to_right_temple",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_temple,
        origin=Origin(xyz=(-0.0080, -0.0621, 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "left_temple_to_left_tip",
        ArticulationType.PRISMATIC,
        parent=left_temple,
        child=left_tip,
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.12,
            lower=0.0,
            upper=0.030,
        ),
    )
    model.articulation(
        "right_temple_to_right_tip",
        ArticulationType.PRISMATIC,
        parent=right_temple,
        child=right_tip,
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.12,
            lower=0.0,
            upper=0.030,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_tip = object_model.get_part("left_tip")
    right_tip = object_model.get_part("right_tip")

    left_hinge = object_model.get_articulation("frame_to_left_temple")
    right_hinge = object_model.get_articulation("frame_to_right_temple")
    left_slide = object_model.get_articulation("left_temple_to_left_tip")
    right_slide = object_model.get_articulation("right_temple_to_right_tip")

    ctx.check(
        "all primary parts are present",
        all(part is not None for part in (frame, left_temple, right_temple, left_tip, right_tip)),
        details="Expected frame, paired temple sleeves, and paired sliding rear sections.",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0, left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_overlap(
            left_tip,
            left_temple,
            axes="x",
            elem_a="insert_blade",
            elem_b="sleeve_shell",
            min_overlap=0.017,
            name="left rear section remains inserted at wear length",
        )
        ctx.expect_overlap(
            right_tip,
            right_temple,
            axes="x",
            elem_a="insert_blade",
            elem_b="sleeve_shell",
            min_overlap=0.017,
            name="right rear section remains inserted at wear length",
        )
        ctx.expect_within(
            left_tip,
            left_temple,
            axes="yz",
            inner_elem="insert_blade",
            outer_elem="sleeve_shell",
            margin=0.0016,
            name="left insert stays centered inside its sleeve",
        )
        ctx.expect_within(
            right_tip,
            right_temple,
            axes="yz",
            inner_elem="insert_blade",
            outer_elem="sleeve_shell",
            margin=0.0016,
            name="right insert stays centered inside its sleeve",
        )

    left_rest = ctx.part_world_position(left_tip)
    right_rest = ctx.part_world_position(right_tip)
    with ctx.pose(
        {
            left_slide: left_slide.motion_limits.upper,
            right_slide: right_slide.motion_limits.upper,
        }
    ):
        ctx.expect_overlap(
            left_tip,
            left_temple,
            axes="x",
            elem_a="insert_blade",
            elem_b="sleeve_shell",
            min_overlap=0.047,
            name="left rear section retracts deeper for packing",
        )
        ctx.expect_overlap(
            right_tip,
            right_temple,
            axes="x",
            elem_a="insert_blade",
            elem_b="sleeve_shell",
            min_overlap=0.047,
            name="right rear section retracts deeper for packing",
        )
        left_packed = ctx.part_world_position(left_tip)
        right_packed = ctx.part_world_position(right_tip)

    ctx.check(
        "rear sections slide forward for packed storage",
        left_rest is not None
        and right_rest is not None
        and left_packed is not None
        and right_packed is not None
        and left_packed[0] > left_rest[0] + 0.025
        and right_packed[0] > right_rest[0] + 0.025,
        details=f"rest_left={left_rest}, packed_left={left_packed}, rest_right={right_rest}, packed_right={right_packed}",
    )

    folded_left_rest = ctx.part_world_position(left_tip)
    folded_right_rest = ctx.part_world_position(right_tip)
    with ctx.pose({left_hinge: 1.45, right_hinge: 1.45}):
        folded_left = ctx.part_world_position(left_tip)
        folded_right = ctx.part_world_position(right_tip)

    ctx.check(
        "temples fold inward toward the frame center",
        folded_left_rest is not None
        and folded_right_rest is not None
        and folded_left is not None
        and folded_right is not None
        and folded_left[1] < folded_left_rest[1] - 0.030
        and folded_right[1] > folded_right_rest[1] + 0.030,
        details=f"left_rest={folded_left_rest}, left_folded={folded_left}, right_rest={folded_right_rest}, right_folded={folded_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
