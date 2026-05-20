from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


Y_AXIS = (0.0, 1.0, 0.0)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _cylinder_along_y(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _add_fork(
    part,
    *,
    joint_x: float,
    joint_z: float,
    prefix: str,
    clear: Material,
    steel: Material,
    metal: Material,
) -> None:
    """Add a pair of clear outer cheek plates and a low metal bridge around a hinge."""
    for side, y in (("pos", 0.058), ("neg", -0.058)):
        part.visual(
            Box((0.150, 0.008, 0.120)),
            origin=Origin(xyz=(joint_x, y, joint_z)),
            material=clear,
            name=f"{prefix}_clear_plate_{side}",
        )
        part.visual(
            _cylinder_along_y(0.024, 0.006),
            origin=_y_cylinder_origin(joint_x, y + (0.007 if y > 0 else -0.007), joint_z),
            material=steel,
            name=f"{prefix}_outer_washer_{side}",
        )

    part.visual(
        Box((0.092, 0.132, 0.012)),
        origin=Origin(xyz=(joint_x, 0.0, joint_z - 0.058)),
        material=metal,
        name=f"{prefix}_lower_bridge",
    )
    part.visual(
        _cylinder_along_y(0.014, 0.134),
        origin=_y_cylinder_origin(joint_x, 0.0, joint_z),
        material=steel,
        name=f"{prefix}_hinge_pin",
    )


def _add_slim_link(
    part,
    *,
    length: float,
    prefix: str,
    clear: Material,
    steel: Material,
    metal: Material,
    accent: Material,
) -> None:
    """Build one slim arm segment with a central hub and a distal fork."""
    body_start = 0.048
    body_end = length - 0.084
    body_len = body_end - body_start
    part.visual(
        Box((body_len, 0.030, 0.022)),
        origin=Origin(xyz=((body_start + body_end) / 2.0, 0.0, 0.0)),
        material=metal,
        name=f"{prefix}_spine",
    )
    part.visual(
        Box((max(body_len - 0.035, 0.050), 0.042, 0.006)),
        origin=Origin(xyz=((body_start + body_end) / 2.0 + 0.010, 0.0, 0.013)),
        material=accent,
        name=f"{prefix}_top_rib",
    )
    part.visual(
        Box((0.100, 0.074, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="proximal_hub",
    )
    part.visual(
        _cylinder_along_y(0.027, 0.078),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material=steel,
        name="proximal_bearing",
    )
    part.visual(
        Box((0.040, 0.122, 0.018)),
        origin=Origin(xyz=(length - 0.090, 0.0, -0.018)),
        material=metal,
        name=f"{prefix}_distal_web",
    )
    _add_fork(
        part,
        joint_x=length,
        joint_z=0.0,
        prefix="distal",
        clear=clear,
        steel=steel,
        metal=metal,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_top_fold_out_arm")

    dark = _mat(model, "matte_charcoal_metal", (0.06, 0.065, 0.07, 1.0))
    steel = _mat(model, "brushed_steel", (0.62, 0.64, 0.66, 1.0))
    clear = _mat(model, "clear_smoked_polycarbonate", (0.55, 0.82, 1.0, 0.34))
    blue = _mat(model, "blue_anodized_rib", (0.08, 0.20, 0.42, 1.0))
    rubber = _mat(model, "black_rubber", (0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.520, 0.280, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark,
        name="base_foot",
    )
    for i, (x, y) in enumerate(((-0.205, -0.105), (-0.205, 0.105), (0.205, -0.105), (0.205, 0.105))):
        base.visual(
            Box((0.070, 0.045, 0.012)),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"rubber_pad_{i}",
        )
    base.visual(
        Box((0.095, 0.110, 0.092)),
        origin=Origin(xyz=(-0.180, 0.0, 0.081)),
        material=dark,
        name="upright_block",
    )
    base.visual(
        Box((0.118, 0.136, 0.016)),
        origin=Origin(xyz=(-0.180, 0.0, 0.125)),
        material=steel,
        name="shoulder_crosshead",
    )
    _add_fork(
        base,
        joint_x=-0.180,
        joint_z=0.170,
        prefix="shoulder",
        clear=clear,
        steel=steel,
        metal=steel,
    )

    link_0 = model.part("link_0")
    _add_slim_link(
        link_0,
        length=0.300,
        prefix="link_0",
        clear=clear,
        steel=steel,
        metal=dark,
        accent=blue,
    )

    link_1 = model.part("link_1")
    _add_slim_link(
        link_1,
        length=0.270,
        prefix="link_1",
        clear=clear,
        steel=steel,
        metal=dark,
        accent=blue,
    )

    link_2 = model.part("link_2")
    _add_slim_link(
        link_2,
        length=0.240,
        prefix="link_2",
        clear=clear,
        steel=steel,
        metal=dark,
        accent=blue,
    )

    platform = model.part("platform")
    platform.visual(
        Box((0.100, 0.074, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="proximal_hub",
    )
    platform.visual(
        _cylinder_along_y(0.027, 0.078),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material=steel,
        name="proximal_bearing",
    )
    platform.visual(
        Box((0.100, 0.048, 0.024)),
        origin=Origin(xyz=(0.095, 0.0, 0.000)),
        material=dark,
        name="platform_neck",
    )
    platform.visual(
        Box((0.140, 0.128, 0.014)),
        origin=Origin(xyz=(0.150, 0.0, 0.018)),
        material=steel,
        name="platform_deck",
    )
    for side, y in (("pos", 0.061), ("neg", -0.061)):
        platform.visual(
            Box((0.120, 0.010, 0.034)),
            origin=Origin(xyz=(0.154, y, 0.036)),
            material=dark,
            name=f"platform_side_lip_{side}",
        )

    limits = MotionLimits(effort=18.0, velocity=1.2, lower=-1.20, upper=1.20)
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(-0.180, 0.0, 0.170), rpy=(0.0, -0.35, 0.0)),
        axis=Y_AXIS,
        motion_limits=limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.300, 0.0, 0.0), rpy=(0.0, 0.25, 0.0)),
        axis=Y_AXIS,
        motion_limits=limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=(0.0, -0.30, 0.0)),
        axis=Y_AXIS,
        motion_limits=limits,
    )
    model.articulation(
        "link_2_to_platform",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=platform,
        origin=Origin(xyz=(0.240, 0.0, 0.0), rpy=(0.0, 0.40, 0.0)),
        axis=Y_AXIS,
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=-1.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("base_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_platform"),
    ]
    ctx.check(
        "four sequential revolute joints",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "joint axes share the arm motion plane",
        all(tuple(round(v, 6) for v in j.axis) == Y_AXIS for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    support_pairs = (
        ("base", "link_0", "shoulder_hinge_pin", "shoulder clear plates capture first hinge"),
        ("link_0", "link_1", "distal_hinge_pin", "first link fork captures second hinge"),
        ("link_1", "link_2", "distal_hinge_pin", "second link fork captures third hinge"),
        ("link_2", "platform", "distal_hinge_pin", "distal link fork captures platform wrist"),
    )
    for parent_name, child_name, pin_elem, check_name in support_pairs:
        ctx.allow_overlap(
            parent_name,
            child_name,
            elem_a=pin_elem,
            elem_b="proximal_bearing",
            reason="The visible steel hinge pin is intentionally captured inside the child's bearing sleeve.",
        )
        ctx.allow_overlap(
            parent_name,
            child_name,
            elem_a=pin_elem,
            elem_b="proximal_hub",
            reason="The hinge pin passes through the solid proxy of the child's hinge boss.",
        )
        ctx.expect_within(
            parent_name,
            child_name,
            axes="xz",
            inner_elem=pin_elem,
            outer_elem="proximal_bearing",
            margin=0.001,
            name=f"{check_name} pin centered in bearing",
        )
        ctx.expect_within(
            parent_name,
            child_name,
            axes="xz",
            inner_elem=pin_elem,
            outer_elem="proximal_hub",
            margin=0.001,
            name=f"{check_name} pin centered in hinge boss",
        )
        ctx.expect_overlap(
            parent_name,
            child_name,
            axes="y",
            elem_a=pin_elem,
            elem_b="proximal_bearing",
            min_overlap=0.060,
            name=f"{check_name} pin passes through bearing width",
        )
        ctx.expect_overlap(
            parent_name,
            child_name,
            axes="y",
            elem_a=pin_elem,
            elem_b="proximal_hub",
            min_overlap=0.060,
            name=f"{check_name} pin passes through hinge boss width",
        )
        ctx.expect_within(
            child_name,
            parent_name,
            axes="y",
            inner_elem="proximal_hub",
            margin=0.002,
            name=f"{check_name} in width",
        )
        ctx.expect_overlap(
            child_name,
            parent_name,
            axes="xz",
            elem_a="proximal_hub",
            min_overlap=0.030,
            name=f"{check_name} in hinge plane",
        )

    platform = object_model.get_part("platform")
    rest_aabb = ctx.part_world_aabb(platform)
    with ctx.pose({"base_to_link_0": 0.55, "link_0_to_link_1": -0.45, "link_1_to_link_2": 0.40}):
        moved_aabb = ctx.part_world_aabb(platform)

    def _center(aabb, idx: int) -> float:
        return (aabb[0][idx] + aabb[1][idx]) * 0.5

    ctx.check(
        "fold-out pose moves distal bracket in xz plane",
        rest_aabb is not None
        and moved_aabb is not None
        and abs(_center(moved_aabb, 1) - _center(rest_aabb, 1)) < 0.001
        and (
            abs(_center(moved_aabb, 0) - _center(rest_aabb, 0)) > 0.020
            or abs(_center(moved_aabb, 2) - _center(rest_aabb, 2)) > 0.020
        ),
        details=f"rest_aabb={rest_aabb}, moved_aabb={moved_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
