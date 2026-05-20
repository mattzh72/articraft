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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BARREL_Y_OFFSET = 0.039
BARREL_Z_OFFSET = -0.026


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _barrel_inertial_origin(side_sign: float) -> Origin:
    return Origin(xyz=(0.0, BARREL_Y_OFFSET * side_sign, -0.022))


def _add_barrel_geometry(
    part,
    *,
    side_sign: float,
    body_material,
    metal_material,
    glass_material,
) -> None:
    y_center = BARREL_Y_OFFSET * side_sign
    z_center = BARREL_Z_OFFSET
    hinge_x = -0.010 if side_sign > 0.0 else 0.010

    part.visual(
        Cylinder(radius=0.0145, length=0.070),
        origin=Origin(xyz=(0.004, y_center, z_center), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_material,
        name="elem_body_tube",
    )
    part.visual(
        Cylinder(radius=0.0165, length=0.020),
        origin=Origin(xyz=(0.034, y_center, z_center), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_material,
        name="elem_objective_bell",
    )
    part.visual(
        Cylinder(radius=0.0122, length=0.028),
        origin=Origin(xyz=(-0.037, y_center, z_center), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_material,
        name="elem_ocular_tube",
    )
    part.visual(
        Cylinder(radius=0.0136, length=0.012),
        origin=Origin(xyz=(-0.051, y_center, z_center), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_material,
        name="elem_eyecup",
    )
    part.visual(
        Box((0.040, 0.026, 0.030)),
        origin=Origin(xyz=(-0.004, y_center, -0.010)),
        material=body_material,
        name="elem_prism_housing",
    )
    part.visual(
        Box((0.022, 0.014, 0.010)),
        origin=Origin(xyz=(-0.008, y_center, 0.008)),
        material=body_material,
        name="elem_roof_ridge",
    )
    part.visual(
        Box((0.028, 0.020, 0.020)),
        origin=Origin(xyz=(-0.006, 0.020 * side_sign, -0.015)),
        material=body_material,
        name="elem_hinge_cheek",
    )
    part.visual(
        Cylinder(radius=0.0085, length=0.016),
        origin=Origin(xyz=(hinge_x, 0.010 * side_sign, -0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_material,
        name="elem_hinge_lug",
    )
    part.visual(
        Cylinder(radius=0.0138, length=0.002),
        origin=Origin(xyz=(0.043, y_center, z_center), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass_material,
        name="elem_objective_lens",
    )
    part.visual(
        Cylinder(radius=0.0108, length=0.002),
        origin=Origin(xyz=(-0.056, y_center, z_center), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass_material,
        name="elem_ocular_lens",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_roof_prism_binoculars")

    armor_black = model.material("armor_black", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.06, 0.06, 0.07, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.35, 0.36, 0.38, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.48, 0.49, 0.52, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.24, 0.36, 0.46, 1.0))

    center_bridge = model.part("center_bridge")
    center_bridge.visual(
        Box((0.042, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=hinge_metal,
        name="elem_pivot_block",
    )
    center_bridge.visual(
        Box((0.018, 0.014, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=armor_black,
        name="elem_bridge_post",
    )
    center_bridge.visual(
        Box((0.022, 0.028, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=armor_black,
        name="elem_bridge_saddle",
    )
    center_bridge.visual(
        Box((0.010, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.013, 0.036)),
        material=hinge_metal,
        name="elem_left_wheel_fork",
    )
    center_bridge.visual(
        Box((0.010, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.013, 0.036)),
        material=hinge_metal,
        name="elem_right_wheel_fork",
    )
    center_bridge.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, 0.060)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    left_barrel = model.part("left_barrel")
    _add_barrel_geometry(
        left_barrel,
        side_sign=-1.0,
        body_material=armor_black,
        metal_material=hinge_metal,
        glass_material=coated_glass,
    )
    left_barrel.inertial = Inertial.from_geometry(
        Box((0.110, 0.040, 0.050)),
        mass=0.18,
        origin=_barrel_inertial_origin(-1.0),
    )

    right_barrel = model.part("right_barrel")
    _add_barrel_geometry(
        right_barrel,
        side_sign=1.0,
        body_material=armor_black,
        metal_material=hinge_metal,
        glass_material=coated_glass,
    )
    right_barrel.inertial = Inertial.from_geometry(
        Box((0.110, 0.040, 0.050)),
        mass=0.18,
        origin=_barrel_inertial_origin(1.0),
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="elem_focus_grip",
    )
    focus_wheel.visual(
        Cylinder(radius=0.0105, length=0.022),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="elem_focus_hub",
    )
    focus_wheel.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(0.0, -0.0065, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="elem_focus_flange_left",
    )
    focus_wheel.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(0.0, 0.0065, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="elem_focus_flange_right",
    )
    focus_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.016),
        mass=0.025,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    diopter_ring = model.part("right_diopter_ring")
    diopter_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0146, -0.007),
            (0.0156, -0.004),
            (0.0158, 0.0),
            (0.0156, 0.004),
            (0.0146, 0.007),
        ],
        [
            (0.0133, -0.007),
            (0.0133, 0.007),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(pi / 2.0)
    diopter_ring.visual(
        _mesh("right_diopter_ring_shell", diopter_shell),
        material=wheel_black,
        name="elem_diopter_ring",
    )
    diopter_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.014),
        mass=0.01,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "bridge_to_left_barrel",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child=left_barrel,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.42,
            upper=0.42,
        ),
    )
    model.articulation(
        "bridge_to_right_barrel",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child=right_barrel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.42,
            upper=0.42,
        ),
    )
    model.articulation(
        "bridge_to_focus_wheel",
        ArticulationType.CONTINUOUS,
        parent=center_bridge,
        child=focus_wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
    )
    model.articulation(
        "right_barrel_to_diopter_ring",
        ArticulationType.CONTINUOUS,
        parent=right_barrel,
        child=diopter_ring,
        origin=Origin(xyz=(-0.040, BARREL_Y_OFFSET, BARREL_Z_OFFSET)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_bridge = object_model.get_part("center_bridge")
    left_barrel = object_model.get_part("left_barrel")
    right_barrel = object_model.get_part("right_barrel")
    focus_wheel = object_model.get_part("focus_wheel")
    diopter_ring = object_model.get_part("right_diopter_ring")

    left_hinge = object_model.get_articulation("bridge_to_left_barrel")
    right_hinge = object_model.get_articulation("bridge_to_right_barrel")
    focus_joint = object_model.get_articulation("bridge_to_focus_wheel")
    diopter_joint = object_model.get_articulation("right_barrel_to_diopter_ring")

    ctx.check(
        "prompt articulations are typed correctly",
        left_hinge.articulation_type == ArticulationType.REVOLUTE
        and right_hinge.articulation_type == ArticulationType.REVOLUTE
        and focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and diopter_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"left={left_hinge.articulation_type}, right={right_hinge.articulation_type}, "
            f"focus={focus_joint.articulation_type}, diopter={diopter_joint.articulation_type}"
        ),
    )
    ctx.check(
        "barrel hinge axes run along the horizontal optical direction",
        left_hinge.axis[0] < -0.9 and right_hinge.axis[0] > 0.9,
        details=f"left_axis={left_hinge.axis}, right_axis={right_hinge.axis}",
    )
    ctx.check(
        "focus wheel and diopter ring rotate on credible axes",
        abs(focus_joint.axis[1]) > 0.9 and abs(diopter_joint.axis[0]) > 0.9,
        details=f"focus_axis={focus_joint.axis}, diopter_axis={diopter_joint.axis}",
    )
    ctx.expect_gap(
        right_barrel,
        left_barrel,
        axis="y",
        min_gap=0.010,
        positive_elem="elem_body_tube",
        negative_elem="elem_body_tube",
        name="barrels stay side-by-side without intersecting at rest",
    )
    ctx.expect_origin_gap(
        focus_wheel,
        center_bridge,
        axis="z",
        min_gap=0.020,
        max_gap=0.050,
        name="focus wheel sits above the central bridge",
    )
    ctx.expect_overlap(
        diopter_ring,
        right_barrel,
        axes="xyz",
        elem_a="elem_diopter_ring",
        elem_b="elem_ocular_tube",
        min_overlap=0.010,
        name="diopter ring sits around the right eyepiece",
    )

    def side_gap() -> float | None:
        left_aabb = ctx.part_element_world_aabb(left_barrel, elem="elem_body_tube")
        right_aabb = ctx.part_element_world_aabb(right_barrel, elem="elem_body_tube")
        if left_aabb is None or right_aabb is None:
            return None
        return right_aabb[0][1] - left_aabb[1][1]

    with ctx.pose({left_hinge: -0.18, right_hinge: -0.18}):
        narrow_gap = side_gap()
    with ctx.pose({left_hinge: 0.18, right_hinge: 0.18}):
        wide_gap = side_gap()

    ctx.check(
        "barrels widen symmetrically for interpupillary adjustment",
        narrow_gap is not None and wide_gap is not None and wide_gap > narrow_gap + 0.010,
        details=f"narrow_gap={narrow_gap}, wide_gap={wide_gap}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
