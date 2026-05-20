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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


FRAME_THICKNESS = 0.0032
LENS_THICKNESS = 0.0018
RIM_OUTER_W = 0.056
RIM_OUTER_H = 0.038
LENS_W = 0.048
LENS_H = 0.030
LENS_CENTER_X = 0.036
OUTER_HINGE_X = 0.065
BRIDGE_KNUCKLE_RADIUS = 0.0017
TEMPLE_KNUCKLE_RADIUS = 0.0016


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rim_mesh():
    outer_profile = rounded_rect_profile(RIM_OUTER_W, RIM_OUTER_H, radius=0.011, corner_segments=8)
    lens_opening = rounded_rect_profile(LENS_W, LENS_H, radius=0.009, corner_segments=8)
    rim_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [lens_opening],
        height=FRAME_THICKNESS,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh("reading_glasses_rim", rim_geom)


def _lens_mesh():
    lens_profile = rounded_rect_profile(LENS_W, LENS_H, radius=0.009, corner_segments=8)
    lens_geom = ExtrudeGeometry(
        lens_profile,
        LENS_THICKNESS,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh("reading_glasses_lens", lens_geom)


def _temple_mesh(name: str, x_bias: float):
    temple_profile = rounded_rect_profile(0.0058, 0.0019, radius=0.0007, corner_segments=4)
    temple_points = [
        (0.0000, -0.0040, 0.0000),
        (0.0015 * x_bias, -0.0300, 0.0000),
        (0.0060 * x_bias, -0.0730, -0.0020),
        (0.0100 * x_bias, -0.1120, -0.0080),
    ]
    return _save_mesh(
        name,
        sweep_profile_along_spline(
            temple_points,
            profile=temple_profile,
            samples_per_segment=16,
            cap_profile=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _add_left_front(part, *, rim_mesh, frame_material, metal_material) -> None:
    part.visual(
        rim_mesh,
        origin=Origin(xyz=(-LENS_CENTER_X, 0.0, 0.0)),
        material=frame_material,
        name="left_rim",
    )
    part.visual(
        Box((0.013, FRAME_THICKNESS, 0.008)),
        origin=Origin(xyz=(-0.0065, 0.0, 0.0005)),
        material=frame_material,
        name="left_bridge_leaf",
    )
    part.visual(
        Box((0.012, FRAME_THICKNESS, 0.006)),
        origin=Origin(xyz=(-0.0095, 0.0, -0.0105)),
        material=frame_material,
        name="left_lower_bridge_bar",
    )
    part.visual(
        Box((0.010, FRAME_THICKNESS, 0.012)),
        origin=Origin(xyz=(-0.0605, 0.0, 0.0)),
        material=frame_material,
        name="left_outer_hinge_block",
    )
    part.visual(
        Cylinder(radius=BRIDGE_KNUCKLE_RADIUS, length=0.0050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_material,
        name="left_bridge_knuckle_center",
    )
    part.visual(
        Cylinder(radius=BRIDGE_KNUCKLE_RADIUS, length=0.0050),
        origin=Origin(xyz=(0.0, 0.0, 0.0067)),
        material=metal_material,
        name="left_bridge_knuckle_top",
    )
    part.visual(
        Cylinder(radius=BRIDGE_KNUCKLE_RADIUS, length=0.0050),
        origin=Origin(xyz=(0.0, 0.0, -0.0060)),
        material=metal_material,
        name="left_bridge_knuckle_bottom",
    )
    part.visual(
        Cylinder(radius=TEMPLE_KNUCKLE_RADIUS, length=0.0070),
        origin=Origin(xyz=(-OUTER_HINGE_X, 0.0, 0.0)),
        material=metal_material,
        name="left_temple_knuckle",
    )
    part.visual(
        Cylinder(radius=0.0012, length=0.0085),
        origin=Origin(xyz=(-0.0112, -0.0018, -0.0158), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_material,
        name="left_nose_pad_arm",
    )
    part.visual(
        Cylinder(radius=0.0040, length=0.0030),
        origin=Origin(xyz=(-0.0148, -0.0033, -0.0186), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="nose_pad_clear",
        name="left_nose_pad",
    )


def _add_right_front(part, *, rim_mesh, frame_material, metal_material) -> None:
    part.visual(
        rim_mesh,
        origin=Origin(xyz=(LENS_CENTER_X, 0.0, 0.0)),
        material=frame_material,
        name="right_rim",
    )
    part.visual(
        Box((0.013, FRAME_THICKNESS, 0.008)),
        origin=Origin(xyz=(0.0065, 0.0, 0.0005)),
        material=frame_material,
        name="right_bridge_leaf",
    )
    part.visual(
        Box((0.012, FRAME_THICKNESS, 0.006)),
        origin=Origin(xyz=(0.0095, 0.0, -0.0105)),
        material=frame_material,
        name="right_lower_bridge_bar",
    )
    part.visual(
        Box((0.010, FRAME_THICKNESS, 0.012)),
        origin=Origin(xyz=(0.0605, 0.0, 0.0)),
        material=frame_material,
        name="right_outer_hinge_block",
    )
    part.visual(
        Cylinder(radius=BRIDGE_KNUCKLE_RADIUS, length=0.0085),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_material,
        name="right_bridge_knuckle_center",
    )
    part.visual(
        Cylinder(radius=TEMPLE_KNUCKLE_RADIUS, length=0.0070),
        origin=Origin(xyz=(OUTER_HINGE_X, 0.0, 0.0)),
        material=metal_material,
        name="right_temple_knuckle",
    )
    part.visual(
        Cylinder(radius=0.0012, length=0.0085),
        origin=Origin(xyz=(0.0112, -0.0018, -0.0158), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_material,
        name="right_nose_pad_arm",
    )
    part.visual(
        Cylinder(radius=0.0040, length=0.0030),
        origin=Origin(xyz=(0.0148, -0.0033, -0.0186), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="nose_pad_clear",
        name="right_nose_pad",
    )


def _add_left_temple(part, *, temple_mesh, temple_material, metal_material) -> None:
    part.visual(
        temple_mesh,
        material=temple_material,
        name="left_temple_arm",
    )
    part.visual(
        Box((0.0075, 0.0060, 0.0135)),
        origin=Origin(xyz=(0.0, -0.0030, 0.0)),
        material=temple_material,
        name="left_temple_hinge_block",
    )
    part.visual(
        Cylinder(radius=TEMPLE_KNUCKLE_RADIUS, length=0.0048),
        origin=Origin(xyz=(0.0, 0.0, 0.0059)),
        material=metal_material,
        name="left_temple_knuckle_top",
    )
    part.visual(
        Cylinder(radius=TEMPLE_KNUCKLE_RADIUS, length=0.0048),
        origin=Origin(xyz=(0.0, 0.0, -0.0059)),
        material=metal_material,
        name="left_temple_knuckle_bottom",
    )
    part.visual(
        Box((0.0068, 0.020, 0.0042)),
        origin=Origin(xyz=(0.0105, -0.102, -0.0085)),
        material=temple_material,
        name="left_temple_tip",
    )


def _add_right_temple(part, *, temple_mesh, temple_material, metal_material) -> None:
    part.visual(
        temple_mesh,
        material=temple_material,
        name="right_temple_arm",
    )
    part.visual(
        Box((0.0075, 0.0060, 0.0135)),
        origin=Origin(xyz=(0.0, -0.0030, 0.0)),
        material=temple_material,
        name="right_temple_hinge_block",
    )
    part.visual(
        Cylinder(radius=TEMPLE_KNUCKLE_RADIUS, length=0.0048),
        origin=Origin(xyz=(0.0, 0.0, 0.0059)),
        material=metal_material,
        name="right_temple_knuckle_top",
    )
    part.visual(
        Cylinder(radius=TEMPLE_KNUCKLE_RADIUS, length=0.0048),
        origin=Origin(xyz=(0.0, 0.0, -0.0059)),
        material=metal_material,
        name="right_temple_knuckle_bottom",
    )
    part.visual(
        Box((0.0068, 0.020, 0.0042)),
        origin=Origin(xyz=(-0.0105, -0.102, -0.0085)),
        material=temple_material,
        name="right_temple_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_reading_glasses")

    frame_burgundy = model.material("frame_burgundy", rgba=(0.44, 0.12, 0.14, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.42, 0.44, 0.48, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.88, 0.94, 1.0, 0.34))
    temple_black = model.material("temple_black", rgba=(0.16, 0.15, 0.16, 1.0))
    nose_pad_clear = model.material("nose_pad_clear", rgba=(0.88, 0.90, 0.92, 0.55))

    rim_mesh = _rim_mesh()
    lens_mesh = _lens_mesh()
    left_temple_mesh = _temple_mesh("left_temple_mesh", x_bias=1.0)
    right_temple_mesh = _temple_mesh("right_temple_mesh", x_bias=-1.0)

    left_front = model.part("left_front")
    _add_left_front(
        left_front,
        rim_mesh=rim_mesh,
        frame_material=frame_burgundy,
        metal_material=gunmetal,
    )
    left_front.inertial = Inertial.from_geometry(
        Box((0.074, 0.012, 0.050)),
        mass=0.020,
        origin=Origin(xyz=(-0.031, -0.001, -0.004)),
    )

    right_front = model.part("right_front")
    _add_right_front(
        right_front,
        rim_mesh=rim_mesh,
        frame_material=frame_burgundy,
        metal_material=gunmetal,
    )
    right_front.inertial = Inertial.from_geometry(
        Box((0.074, 0.012, 0.050)),
        mass=0.020,
        origin=Origin(xyz=(0.031, -0.001, -0.004)),
    )

    left_lens = model.part("left_lens")
    left_lens.visual(
        lens_mesh,
        material=lens_glass,
        name="left_lens_body",
    )
    left_lens.inertial = Inertial.from_geometry(
        Box((LENS_W, LENS_THICKNESS, LENS_H)),
        mass=0.004,
        origin=Origin(),
    )

    right_lens = model.part("right_lens")
    right_lens.visual(
        lens_mesh,
        material=lens_glass,
        name="right_lens_body",
    )
    right_lens.inertial = Inertial.from_geometry(
        Box((LENS_W, LENS_THICKNESS, LENS_H)),
        mass=0.004,
        origin=Origin(),
    )

    left_temple = model.part("left_temple")
    _add_left_temple(
        left_temple,
        temple_mesh=left_temple_mesh,
        temple_material=temple_black,
        metal_material=gunmetal,
    )
    left_temple.inertial = Inertial.from_geometry(
        Box((0.020, 0.124, 0.018)),
        mass=0.008,
        origin=Origin(xyz=(0.005, -0.060, -0.004)),
    )

    right_temple = model.part("right_temple")
    _add_right_temple(
        right_temple,
        temple_mesh=right_temple_mesh,
        temple_material=temple_black,
        metal_material=gunmetal,
    )
    right_temple.inertial = Inertial.from_geometry(
        Box((0.020, 0.124, 0.018)),
        mass=0.008,
        origin=Origin(xyz=(-0.005, -0.060, -0.004)),
    )

    model.articulation(
        "bridge_hinge",
        ArticulationType.REVOLUTE,
        parent=left_front,
        child=right_front,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.30,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(155.0),
        ),
    )
    model.articulation(
        "left_lens_mount",
        ArticulationType.FIXED,
        parent=left_front,
        child=left_lens,
        origin=Origin(xyz=(-LENS_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "right_lens_mount",
        ArticulationType.FIXED,
        parent=right_front,
        child=right_lens,
        origin=Origin(xyz=(LENS_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=left_front,
        child=left_temple,
        origin=Origin(xyz=(-OUTER_HINGE_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=5.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=right_front,
        child=right_temple,
        origin=Origin(xyz=(OUTER_HINGE_X, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=5.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_front = object_model.get_part("left_front")
    right_front = object_model.get_part("right_front")
    left_lens = object_model.get_part("left_lens")
    right_lens = object_model.get_part("right_lens")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")

    bridge_hinge = object_model.get_articulation("bridge_hinge")
    left_temple_hinge = object_model.get_articulation("left_temple_hinge")
    right_temple_hinge = object_model.get_articulation("right_temple_hinge")

    ctx.check(
        "primary assemblies exist",
        all(
            part is not None
            for part in (left_front, right_front, left_lens, right_lens, left_temple, right_temple)
        ),
        details="Expected both half-fronts, both lenses, and both temples.",
    )

    ctx.expect_gap(
        right_front,
        left_front,
        axis="x",
        positive_elem="right_bridge_leaf",
        negative_elem="left_bridge_leaf",
        max_gap=0.001,
        max_penetration=0.0,
        name="bridge leaves meet cleanly when unfolded",
    )
    ctx.expect_gap(
        left_front,
        left_temple,
        axis="y",
        positive_elem="left_rim",
        negative_elem="left_temple_arm",
        min_gap=0.0005,
        name="left temple opens behind left rim",
    )
    ctx.expect_gap(
        right_front,
        right_temple,
        axis="y",
        positive_elem="right_rim",
        negative_elem="right_temple_arm",
        min_gap=0.0005,
        name="right temple opens behind right rim",
    )
    ctx.expect_within(
        left_lens,
        left_front,
        axes="xz",
        inner_elem="left_lens_body",
        outer_elem="left_rim",
        margin=0.0,
        name="left lens stays within the left rim envelope",
    )
    ctx.expect_within(
        right_lens,
        right_front,
        axes="xz",
        inner_elem="right_lens_body",
        outer_elem="right_rim",
        margin=0.0,
        name="right lens stays within the right rim envelope",
    )

    open_right_center = _aabb_center(ctx.part_world_aabb(right_front))
    open_left_temple_center = _aabb_center(ctx.part_world_aabb(left_temple))
    open_right_temple_center = _aabb_center(ctx.part_world_aabb(right_temple))

    with ctx.pose({bridge_hinge: math.radians(120.0)}):
        folded_right_center = _aabb_center(ctx.part_world_aabb(right_front))
    ctx.check(
        "bridge hinge folds the right half-front backward",
        open_right_center is not None
        and folded_right_center is not None
        and folded_right_center[1] < open_right_center[1] - 0.020,
        details=f"open_center={open_right_center}, folded_center={folded_right_center}",
    )

    with ctx.pose({left_temple_hinge: math.radians(82.0)}):
        folded_left_temple_center = _aabb_center(ctx.part_world_aabb(left_temple))
    ctx.check(
        "left temple folds inward toward the bridge",
        open_left_temple_center is not None
        and folded_left_temple_center is not None
        and folded_left_temple_center[0] > open_left_temple_center[0] + 0.030
        and folded_left_temple_center[1] > open_left_temple_center[1] + 0.030,
        details=f"open_center={open_left_temple_center}, folded_center={folded_left_temple_center}",
    )

    with ctx.pose({right_temple_hinge: math.radians(82.0)}):
        folded_right_temple_center = _aabb_center(ctx.part_world_aabb(right_temple))
    ctx.check(
        "right temple folds inward toward the bridge",
        open_right_temple_center is not None
        and folded_right_temple_center is not None
        and folded_right_temple_center[0] < open_right_temple_center[0] - 0.030
        and folded_right_temple_center[1] > open_right_temple_center[1] + 0.030,
        details=f"open_center={open_right_temple_center}, folded_center={folded_right_temple_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
