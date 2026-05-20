from __future__ import annotations

import math

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


STEEL_RADIUS = 0.008
LIGHT_ROD_RADIUS = 0.006


def _rod(
    part,
    name: str,
    *,
    center: tuple[float, float, float],
    length: float,
    radius: float,
    axis: str,
    material: Material,
) -> None:
    """Add a round drying-rack tube aligned to one local axis."""
    rpy = {
        "x": (0.0, math.pi / 2.0, 0.0),
        "y": (-math.pi / 2.0, 0.0, 0.0),
        "z": (0.0, 0.0, 0.0),
    }[axis]
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _slanted_rod_yz(
    part,
    name: str,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
) -> None:
    """Add a cylinder for the lower folding frame's side legs in a Y-Z plane."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dx) > 1e-9:
        raise ValueError("_slanted_rod_yz only supports rods in a Y-Z plane")
    # Local cylinder +Z rotated about X gives vector (0, -sin(theta), cos(theta)).
    theta = math.atan2(-dy, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(theta, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_wire_panel(
    part,
    *,
    sign: float,
    steel: Material,
    plastic: Material,
) -> None:
    """Build one hinged side wing in the child frame; local origin is hinge line."""
    length = 1.02
    inner_y = sign * 0.052
    outer_y = sign * 0.390
    center_y = sign * 0.221

    # A small plastic tongue starts just outside the central top cap, implying
    # the real hinge is hidden inside that cap rather than hanging proud.
    part.visual(
        Box((1.055, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, sign * 0.049, 0.012)),
        material=plastic,
        name="hinge_tongue",
    )
    _rod(part, "inner_rail", center=(0.0, inner_y, 0.0), length=length, radius=0.009, axis="x", material=steel)
    _rod(part, "outer_rail", center=(0.0, outer_y, 0.0), length=length, radius=0.009, axis="x", material=steel)

    for x, suffix in ((-0.51, "end_rail_0"), (0.51, "end_rail_1")):
        _rod(
            part,
            suffix,
            center=(x, center_y, 0.0),
            length=abs(outer_y - inner_y),
            radius=0.008,
            axis="y",
            material=steel,
        )

    for i, y_abs in enumerate((0.125, 0.200, 0.275, 0.350)):
        _rod(
            part,
            f"hanging_rail_{i}",
            center=(0.0, sign * y_abs, 0.006),
            length=1.020,
            radius=LIGHT_ROD_RADIUS,
            axis="x",
            material=steel,
        )

    # Soft caps at the exposed corners make the wing read as a consumer rack,
    # not a bare wire sketch.
    for i, x in enumerate((-0.51, 0.51)):
        part.visual(
            Box((0.050, 0.030, 0.024)),
            origin=Origin(xyz=(x, outer_y, 0.0)),
            material=plastic,
            name=f"corner_cap_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    steel = model.material("white_powder_coated_steel", rgba=(0.92, 0.94, 0.93, 1.0))
    plastic = model.material("warm_gray_plastic", rgba=(0.50, 0.52, 0.50, 1.0))
    rubber = model.material("dark_rubber_feet", rgba=(0.04, 0.045, 0.045, 1.0))

    central = model.part("central_frame")
    rack_z = 0.858
    rack_length = 1.10
    rack_width = 0.430

    # Concealed hinge/top cap strips; wing hinge hardware sits under these
    # caps, giving the rack a clean unbroken top silhouette.
    central.visual(
        Box((1.160, 0.080, 0.034)),
        origin=Origin(xyz=(0.0, 0.265, rack_z + 0.015)),
        material=plastic,
        name="top_cap_0",
    )
    central.visual(
        Box((1.160, 0.080, 0.034)),
        origin=Origin(xyz=(0.0, -0.265, rack_z + 0.015)),
        material=plastic,
        name="top_cap_1",
    )

    central.visual(
        Box((1.100, 0.066, 0.030)),
        origin=Origin(xyz=(0.0, -0.195, rack_z - 0.018)),
        material=plastic,
        name="underside_pivot_cover",
    )

    # Main rectangular center rack.
    for sign, name in ((1.0, "side_rail_0"), (-1.0, "side_rail_1")):
        _rod(
            central,
            name,
            center=(0.0, sign * rack_width / 2.0, rack_z),
            length=rack_length,
            radius=0.010,
            axis="x",
            material=steel,
        )

    for x, name in ((-rack_length / 2.0, "end_rail_0"), (rack_length / 2.0, "end_rail_1")):
        _rod(
            central,
            name,
            center=(x, 0.0, rack_z),
            length=rack_width,
            radius=0.010,
            axis="y",
            material=steel,
        )

    for i, y in enumerate((-0.160, -0.090, -0.020, 0.050, 0.120, 0.190)):
        _rod(
            central,
            f"hanging_rail_{i}",
            center=(0.0, y, rack_z + 0.004),
            length=1.080,
            radius=LIGHT_ROD_RADIUS,
            axis="x",
            material=steel,
        )

    # Small shadow slots make the concealed hinge lines legible without exposing
    # bulky hinge barrels outside the cap.
    for sign, name in ((1.0, "hinge_shadow_0"), (-1.0, "hinge_shadow_1")):
        central.visual(
            Box((1.070, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, sign * 0.292, rack_z + 0.001)),
            material=rubber,
            name=name,
        )

    wing_0 = model.part("wing_0")
    _add_wire_panel(wing_0, sign=1.0, steel=steel, plastic=plastic)

    wing_1 = model.part("wing_1")
    _add_wire_panel(wing_1, sign=-1.0, steel=steel, plastic=plastic)

    lower_support = model.part("lower_support")
    top = (0.0, 0.000, 0.000)
    bottom = (0.0, 0.185, -0.565)
    post_x = 0.500

    _rod(
        lower_support,
        "top_rail",
        center=(0.0, top[1], top[2]),
        length=1.040,
        radius=0.009,
        axis="x",
        material=steel,
    )
    _rod(
        lower_support,
        "bottom_rail",
        center=(0.0, bottom[1], bottom[2]),
        length=1.040,
        radius=0.009,
        axis="x",
        material=steel,
    )
    for sign, name in ((-1.0, "side_post_0"), (1.0, "side_post_1")):
        _slanted_rod_yz(
            lower_support,
            name,
            start=(sign * post_x, top[1], top[2]),
            end=(sign * post_x, bottom[1], bottom[2]),
            radius=0.009,
            material=steel,
        )

    for i, t in enumerate((0.25, 0.50, 0.75)):
        y = top[1] + (bottom[1] - top[1]) * t
        z = top[2] + (bottom[2] - top[2]) * t
        _rod(
            lower_support,
            f"hanging_rail_{i}",
            center=(0.0, y, z),
            length=1.020,
            radius=LIGHT_ROD_RADIUS,
            axis="x",
            material=steel,
        )

    for i, sign in enumerate((-1.0, 1.0)):
        lower_support.visual(
            Box((0.085, 0.050, 0.026)),
            origin=Origin(xyz=(sign * post_x, bottom[1] + 0.010, bottom[2] - 0.006)),
            material=rubber,
            name=f"foot_{i}",
        )

    wing_limits = MotionLimits(effort=7.0, velocity=2.5, lower=0.0, upper=1.70)
    model.articulation(
        "central_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_0,
        origin=Origin(xyz=(0.0, 0.265, rack_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=wing_limits,
    )
    model.articulation(
        "central_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_1,
        origin=Origin(xyz=(0.0, -0.265, rack_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=wing_limits,
    )
    model.articulation(
        "central_to_lower_support",
        ArticulationType.REVOLUTE,
        parent=central,
        child=lower_support,
        origin=Origin(xyz=(0.0, -0.195, rack_z - 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    support = object_model.get_part("lower_support")
    hinge_0 = object_model.get_articulation("central_to_wing_0")
    hinge_1 = object_model.get_articulation("central_to_wing_1")
    support_hinge = object_model.get_articulation("central_to_lower_support")

    ctx.check(
        "primary mechanisms are revolute",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (hinge_0, hinge_1, support_hinge)),
    )
    ctx.expect_overlap(
        wing_0,
        central,
        axes="x",
        min_overlap=0.95,
        elem_a="inner_rail",
        elem_b="top_cap_0",
        name="wing_0 hinge line spans the top cap",
    )
    ctx.expect_overlap(
        wing_1,
        central,
        axes="x",
        min_overlap=0.95,
        elem_a="inner_rail",
        elem_b="top_cap_1",
        name="wing_1 hinge line spans the top cap",
    )
    ctx.expect_gap(
        wing_0,
        central,
        axis="y",
        min_gap=0.0,
        max_gap=0.040,
        positive_elem="hinge_tongue",
        negative_elem="top_cap_0",
        name="wing_0 hinge tongue is tucked beside cap",
    )
    ctx.expect_gap(
        central,
        wing_1,
        axis="y",
        min_gap=0.0,
        max_gap=0.040,
        positive_elem="top_cap_1",
        negative_elem="hinge_tongue",
        name="wing_1 hinge tongue is tucked beside cap",
    )
    ctx.expect_gap(
        central,
        support,
        axis="z",
        min_gap=0.0,
        max_gap=0.040,
        positive_elem="underside_pivot_cover",
        negative_elem="top_rail",
        name="lower support hinge sits under the top cap cover",
    )
    ctx.expect_overlap(
        support,
        central,
        axes="x",
        min_overlap=0.95,
        elem_a="top_rail",
        elem_b="underside_pivot_cover",
        name="lower support hinge spans the central rack",
    )

    def _elem_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    rest_wing_0_z = _elem_center_z(wing_0, "outer_rail")
    rest_wing_1_z = _elem_center_z(wing_1, "outer_rail")
    rest_foot_z = _elem_center_z(support, "foot_0")

    with ctx.pose({hinge_0: 1.10, hinge_1: 1.10, support_hinge: 1.10}):
        raised_wing_0_z = _elem_center_z(wing_0, "outer_rail")
        raised_wing_1_z = _elem_center_z(wing_1, "outer_rail")
        folded_foot_z = _elem_center_z(support, "foot_0")

    ctx.check(
        "wing_0 folds upward from its side hinge",
        rest_wing_0_z is not None and raised_wing_0_z is not None and raised_wing_0_z > rest_wing_0_z + 0.20,
        details=f"rest={rest_wing_0_z}, raised={raised_wing_0_z}",
    )
    ctx.check(
        "wing_1 folds upward from its side hinge",
        rest_wing_1_z is not None and raised_wing_1_z is not None and raised_wing_1_z > rest_wing_1_z + 0.20,
        details=f"rest={rest_wing_1_z}, raised={raised_wing_1_z}",
    )
    ctx.check(
        "lower support frame folds upward",
        rest_foot_z is not None and folded_foot_z is not None and folded_foot_z > rest_foot_z + 0.25,
        details=f"rest={rest_foot_z}, folded={folded_foot_z}",
    )

    return ctx.report()


object_model = build_object_model()
