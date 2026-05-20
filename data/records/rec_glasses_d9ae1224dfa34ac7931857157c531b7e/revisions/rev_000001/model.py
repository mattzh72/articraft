from __future__ import annotations

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


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_x(center: tuple[float, float, float], radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length, both=True).translate(center)


def _cyl_z(center: tuple[float, float, float], radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length, both=True).translate(center)


def _oval_disc(
    center_x: float,
    center_z: float,
    radius_x: float,
    radius_z: float,
    depth_y: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .ellipse(radius_x, radius_z)
        .extrude(depth_y, both=True)
    )


def _oval_ring(
    center_x: float,
    center_z: float,
    radius_x: float,
    radius_z: float,
    rim_width: float,
    depth_y: float,
) -> cq.Workplane:
    outer = _oval_disc(center_x, center_z, radius_x, radius_z, depth_y)
    inner = _oval_disc(
        center_x,
        center_z,
        radius_x - rim_width,
        radius_z - rim_width * 0.88,
        depth_y * 3.0,
    )
    return outer.cut(inner)


def _primary_frame_shape() -> cq.Workplane:
    lens_dx = 0.036
    body = _oval_ring(-lens_dx, 0.0, 0.032, 0.024, 0.0048, 0.006)
    body = body.union(_oval_ring(lens_dx, 0.0, 0.032, 0.024, 0.0048, 0.006))

    # Nose bridge and continuous brow bar tie the two rims into one front.
    body = body.union(_box((0.0, 0.0, -0.003), (0.020, 0.006, 0.008)))
    body = body.union(_box((0.0, 0.0, 0.0235), (0.138, 0.006, 0.006)))

    # Outer corner hinge housings for the temple arms.
    for sx in (-1.0, 1.0):
        body = body.union(_box((sx * 0.072, 0.001, 0.005), (0.014, 0.010, 0.024)))
        body = body.union(_box((sx * 0.080, 0.013, 0.005), (0.006, 0.014, 0.017)))

    # Two small forward hinge bridges for the flip-up tinted front.
    for sx in (-1.0, 1.0):
        body = body.union(_box((sx * 0.034, -0.0085, 0.0275), (0.008, 0.014, 0.012)))
        body = body.union(_cyl_x((sx * 0.034, -0.0132, 0.031), 0.0021, 0.008))

    return body


def _clear_lenses_shape() -> cq.Workplane:
    lens_dx = 0.036
    left = _oval_disc(-lens_dx, -0.001, 0.0284, 0.0203, 0.0016)
    return left.union(_oval_disc(lens_dx, -0.001, 0.0284, 0.0203, 0.0016))


def _tinted_frame_shape() -> cq.Workplane:
    lens_dx = 0.036
    body = _oval_ring(-lens_dx, -0.027, 0.031, 0.0225, 0.0032, 0.002)
    body = body.union(_oval_ring(lens_dx, -0.027, 0.031, 0.0225, 0.0032, 0.002))
    body = body.union(_box((0.0, 0.0, -0.027), (0.018, 0.004, 0.006)))
    body = body.union(_box((0.0, 0.0, -0.006), (0.128, 0.004, 0.005)))

    # Hinge knuckles and small ears reaching up to the common flip-up axis.
    for sx in (-1.0, 1.0):
        body = body.union(_cyl_x((sx * 0.034, 0.0, 0.0), 0.0025, 0.012))
        body = body.union(_box((sx * 0.034, 0.0, -0.0045), (0.006, 0.004, 0.009)))
    return body


def _tinted_lenses_shape() -> cq.Workplane:
    lens_dx = 0.036
    left = _oval_disc(-lens_dx, -0.027, 0.0278, 0.0193, 0.002)
    return left.union(_oval_disc(lens_dx, -0.027, 0.0278, 0.0193, 0.002))


def _temple_shape(side: float) -> cq.Workplane:
    """Temple arm in its own hinge frame. side=+1 for right, -1 for left."""
    inward = -side
    body = _cyl_z((0.0, 0.0, 0.0), 0.0025, 0.018)
    body = body.union(_box((inward * 0.0035, 0.007, 0.0), (0.009, 0.016, 0.006)))
    body = body.union(_box((inward * 0.008, 0.070, -0.001), (0.006, 0.132, 0.005)))
    # Downturned ear hook at the rear end.
    body = body.union(_box((inward * 0.012, 0.139, -0.017), (0.006, 0.025, 0.032)))
    return body


def _temple_tip_shape(side: float) -> cq.Workplane:
    inward = -side
    return _box((inward * 0.012, 0.151, -0.028), (0.007, 0.020, 0.014))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_up_driving_glasses")

    black = model.material("satin_black_acetate", rgba=(0.015, 0.014, 0.012, 1.0))
    dark_rubber = model.material("soft_black_tips", rgba=(0.005, 0.005, 0.005, 1.0))
    clear_lens = model.material("clear_primary_lens", rgba=(0.80, 0.92, 1.0, 0.34))
    amber_lens = model.material("amber_tinted_lens", rgba=(0.95, 0.48, 0.08, 0.50))
    main_frame = model.part("main_frame")
    main_frame.visual(
        mesh_from_cadquery(_primary_frame_shape(), "main_frame_shell", tolerance=0.00045),
        material=black,
        name="primary_rims",
    )
    main_frame.visual(
        mesh_from_cadquery(_clear_lenses_shape(), "primary_lenses", tolerance=0.00045),
        material=clear_lens,
        name="primary_lenses",
    )
    tinted_front = model.part("tinted_front")
    tinted_front.visual(
        mesh_from_cadquery(_tinted_frame_shape(), "tinted_front_frame", tolerance=0.00045),
        material=black,
        name="tinted_frame",
    )
    tinted_front.visual(
        mesh_from_cadquery(_tinted_lenses_shape(), "tinted_lenses", tolerance=0.00045),
        material=amber_lens,
        name="tinted_lenses",
    )

    left_temple = model.part("left_temple")
    left_temple.visual(
        mesh_from_cadquery(_temple_shape(-1.0), "left_temple_arm", tolerance=0.00045),
        material=black,
        name="temple_arm",
    )
    left_temple.visual(
        mesh_from_cadquery(_temple_tip_shape(-1.0), "left_temple_tip", tolerance=0.00045),
        material=dark_rubber,
        name="ear_tip",
    )

    right_temple = model.part("right_temple")
    right_temple.visual(
        mesh_from_cadquery(_temple_shape(1.0), "right_temple_arm", tolerance=0.00045),
        material=black,
        name="temple_arm",
    )
    right_temple.visual(
        mesh_from_cadquery(_temple_tip_shape(1.0), "right_temple_tip", tolerance=0.00045),
        material=dark_rubber,
        name="ear_tip",
    )

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=left_temple,
        origin=Origin(xyz=(-0.080, 0.022, 0.005)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=2.5, lower=0.0, upper=1.65),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=right_temple,
        origin=Origin(xyz=(0.080, 0.022, 0.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=2.5, lower=0.0, upper=1.65),
    )
    model.articulation(
        "front_flip_hinge",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=tinted_front,
        origin=Origin(xyz=(0.0, -0.018, 0.031)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.22, velocity=2.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_frame = object_model.get_part("main_frame")
    tinted_front = object_model.get_part("tinted_front")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    flip = object_model.get_articulation("front_flip_hinge")
    left_hinge = object_model.get_articulation("left_temple_hinge")
    right_hinge = object_model.get_articulation("right_temple_hinge")

    ctx.check(
        "three user-facing revolute hinges",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.expect_gap(
        main_frame,
        tinted_front,
        axis="y",
        min_gap=-0.002,
        max_gap=0.004,
        name="flip front sits just ahead of the primary rims",
    )
    ctx.expect_overlap(
        tinted_front,
        main_frame,
        axes="x",
        min_overlap=0.11,
        name="tinted front spans both primary lenses",
    )

    rest_front = ctx.part_world_aabb(tinted_front)
    rest_left = ctx.part_world_aabb(left_temple)
    rest_right = ctx.part_world_aabb(right_temple)

    with ctx.pose({flip: 1.45}):
        raised_front = ctx.part_world_aabb(tinted_front)

    with ctx.pose({left_hinge: 1.55, right_hinge: 1.55}):
        folded_left = ctx.part_world_aabb(left_temple)
        folded_right = ctx.part_world_aabb(right_temple)

    ctx.check(
        "flip-up front raises the tinted lenses",
        rest_front is not None
        and raised_front is not None
        and raised_front[0][2] > rest_front[0][2] + 0.020,
        details=f"rest={rest_front}, raised={raised_front}",
    )
    ctx.check(
        "temples start extended backward",
        rest_left is not None
        and rest_right is not None
        and rest_left[1][1] > 0.14
        and rest_right[1][1] > 0.14,
        details=f"left={rest_left}, right={rest_right}",
    )
    ctx.check(
        "temples fold inward across the frame",
        folded_left is not None
        and folded_right is not None
        and folded_left[1][0] > 0.020
        and folded_right[0][0] < -0.020,
        details=f"left={folded_left}, right={folded_right}",
    )

    return ctx.report()


object_model = build_object_model()
