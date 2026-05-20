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
    Part,
    TestContext,
    TestReport,
)


def _x_tube(part: Part, name: str, *, x: float, y: float, z: float, length: float, radius: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_tube(part: Part, name: str, *, x: float, y: float, z: float, length: float, radius: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _z_tube(part: Part, name: str, *, x: float, y: float, z: float, length: float, radius: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def _box(part: Part, name: str, *, x: float, y: float, z: float, size: tuple[float, float, float], material: Material) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    white = model.material("white_powder_coat", rgba=(0.94, 0.96, 0.96, 1.0))
    hinge = model.material("grey_hinge_plastic", rgba=(0.38, 0.40, 0.42, 1.0))
    rubber = model.material("blue_rubber_feet", rgba=(0.08, 0.25, 0.70, 1.0))

    tube_r = 0.012
    rail_r = 0.008
    hinge_r = 0.016

    central = model.part("central_frame")

    # Household-scale central drying rack: a rigid top grid carried by four legs.
    _x_tube(central, "rear_top_tube", x=0.0, y=-0.275, z=0.820, length=1.120, radius=tube_r, material=white)
    _x_tube(central, "front_top_tube", x=0.0, y=0.275, z=0.820, length=1.120, radius=tube_r, material=white)
    _y_tube(central, "end_tube_0", x=-0.560, y=0.0, z=0.820, length=0.550, radius=tube_r, material=white)
    _y_tube(central, "end_tube_1", x=0.560, y=0.0, z=0.820, length=0.550, radius=tube_r, material=white)
    for i, y in enumerate((-0.165, -0.055, 0.055, 0.165)):
        _x_tube(central, f"center_rail_{i}", x=0.0, y=y, z=0.834, length=1.120, radius=rail_r, material=white)

    for x in (-0.560, 0.560):
        for y in (-0.275, 0.275):
            _z_tube(central, f"leg_{x}_{y}", x=x, y=y, z=0.405, length=0.830, radius=tube_r, material=white)

    _x_tube(central, "rear_foot", x=0.0, y=-0.275, z=0.018, length=1.150, radius=0.015, material=white)
    _x_tube(central, "front_foot", x=0.0, y=0.275, z=0.018, length=1.150, radius=0.015, material=white)
    for i, (x, y) in enumerate(((-0.585, -0.275), (0.585, -0.275), (-0.585, 0.275), (0.585, 0.275))):
        central.visual(
            Cylinder(radius=0.021, length=0.028),
            origin=Origin(xyz=(x, y, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    # Exposed hinge hardware on both long sides.  The grey leaves connect the
    # alternating barrel knuckles back into the central tube so the wings read
    # as pinned rather than floating beside the rack.
    for side, y_axis, y_leaf in (("front", 0.315, 0.279), ("rear", -0.315, -0.279)):
        _x_tube(central, f"{side}_hinge_knuckle_0", x=-0.330, y=y_axis, z=0.820, length=0.220, radius=hinge_r, material=hinge)
        _x_tube(central, f"{side}_hinge_knuckle_1", x=0.330, y=y_axis, z=0.820, length=0.220, radius=hinge_r, material=hinge)
        _box(central, f"{side}_hinge_leaf_0", x=-0.200, y=y_leaf, z=0.820, size=(0.460, 0.040, 0.008), material=hinge)
        _box(central, f"{side}_hinge_leaf_1", x=0.200, y=y_leaf, z=0.820, size=(0.460, 0.040, 0.008), material=hinge)

    # Lower support hinge line is below the rack and tied into the rear legs by
    # a crossbar plus short yokes.
    _x_tube(central, "lower_hinge_crossbar", x=0.0, y=-0.275, z=0.520, length=1.120, radius=0.010, material=white)
    for i, x in enumerate((-0.330, 0.330)):
        _y_tube(central, f"lower_hinge_yoke_{i}", x=x, y=-0.2275, z=0.520, length=0.095, radius=0.008, material=hinge)
        _x_tube(central, f"lower_hinge_knuckle_{i}", x=(0.300 if x > 0 else -0.300), y=-0.180, z=0.520, length=0.180, radius=0.014, material=hinge)

    front_wing = model.part("wing_0")
    _x_tube(front_wing, "hinge_knuckle", x=0.0, y=0.0, z=0.0, length=0.440, radius=hinge_r, material=hinge)
    _box(front_wing, "hinge_leaf", x=0.0, y=0.051, z=0.0, size=(0.460, 0.070, 0.008), material=hinge)
    _x_tube(front_wing, "inner_rail", x=0.0, y=0.095, z=0.000, length=1.000, radius=tube_r, material=white)
    _x_tube(front_wing, "outer_rail", x=0.0, y=0.480, z=0.000, length=0.920, radius=tube_r, material=white)
    for i, x in enumerate((-0.460, 0.460)):
        _y_tube(front_wing, f"side_tube_{i}", x=x, y=0.280, z=0.000, length=0.400, radius=tube_r, material=white)
    for i, y in enumerate((0.180, 0.280, 0.380)):
        _x_tube(front_wing, f"drying_rail_{i}", x=0.0, y=y, z=0.014, length=0.920, radius=rail_r, material=white)

    rear_wing = model.part("wing_1")
    _x_tube(rear_wing, "hinge_knuckle", x=0.0, y=0.0, z=0.0, length=0.440, radius=hinge_r, material=hinge)
    _box(rear_wing, "hinge_leaf", x=0.0, y=-0.051, z=0.0, size=(0.460, 0.070, 0.008), material=hinge)
    _x_tube(rear_wing, "inner_rail", x=0.0, y=-0.095, z=0.000, length=1.000, radius=tube_r, material=white)
    _x_tube(rear_wing, "outer_rail", x=0.0, y=-0.480, z=0.000, length=0.920, radius=tube_r, material=white)
    for i, x in enumerate((-0.460, 0.460)):
        _y_tube(rear_wing, f"side_tube_{i}", x=x, y=-0.280, z=0.000, length=0.400, radius=tube_r, material=white)
    for i, y in enumerate((-0.180, -0.280, -0.380)):
        _x_tube(rear_wing, f"drying_rail_{i}", x=0.0, y=y, z=0.014, length=0.920, radius=rail_r, material=white)

    lower = model.part("lower_frame")
    _x_tube(lower, "hinge_knuckle", x=0.0, y=0.0, z=0.0, length=0.420, radius=0.014, material=hinge)
    _box(lower, "hinge_leaf", x=0.0, y=0.049, z=0.0, size=(0.900, 0.070, 0.008), material=hinge)
    _x_tube(lower, "inner_rail", x=0.0, y=0.070, z=0.000, length=0.880, radius=tube_r, material=white)
    _x_tube(lower, "outer_rail", x=0.0, y=0.420, z=0.000, length=0.880, radius=tube_r, material=white)
    for i, x in enumerate((-0.440, 0.440)):
        _y_tube(lower, f"side_tube_{i}", x=x, y=0.245, z=0.000, length=0.350, radius=tube_r, material=white)
    for i, y in enumerate((0.155, 0.245, 0.335)):
        _x_tube(lower, f"hanging_rail_{i}", x=0.0, y=y, z=0.014, length=0.880, radius=rail_r, material=white)

    model.articulation(
        "center_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=front_wing,
        origin=Origin(xyz=(0.0, 0.315, 0.820)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=8.0, velocity=2.5),
    )
    model.articulation(
        "center_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=rear_wing,
        origin=Origin(xyz=(0.0, -0.315, 0.820)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=8.0, velocity=2.5),
    )
    model.articulation(
        "center_to_lower",
        ArticulationType.REVOLUTE,
        parent=central,
        child=lower,
        origin=Origin(xyz=(0.0, -0.180, 0.520)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=8.0, velocity=2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    lower = object_model.get_part("lower_frame")
    wing_0_joint = object_model.get_articulation("center_to_wing_0")
    wing_1_joint = object_model.get_articulation("center_to_wing_1")
    lower_joint = object_model.get_articulation("center_to_lower")

    ctx.check(
        "three revolute fold joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (wing_0_joint, wing_1_joint, lower_joint)),
    )
    ctx.expect_contact(central, wing_0, contact_tol=0.002, name="front wing hinge is supported")
    ctx.expect_contact(central, wing_1, contact_tol=0.002, name="rear wing hinge is supported")
    ctx.expect_contact(central, lower, contact_tol=0.002, name="lower frame hinge is supported")
    ctx.expect_overlap(central, wing_0, axes="x", min_overlap=0.20, name="front wing shares hinge span")
    ctx.expect_overlap(central, wing_1, axes="x", min_overlap=0.20, name="rear wing shares hinge span")
    ctx.expect_overlap(central, lower, axes="x", min_overlap=0.20, name="lower frame shares hinge span")

    def elem_z(part: Part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    wing_0_rest = elem_z(wing_0, "outer_rail")
    with ctx.pose({wing_0_joint: 1.20}):
        wing_0_folded = elem_z(wing_0, "outer_rail")
    ctx.check(
        "front wing folds upward",
        wing_0_rest is not None and wing_0_folded is not None and wing_0_folded > wing_0_rest + 0.30,
        details=f"rest={wing_0_rest}, folded={wing_0_folded}",
    )

    wing_1_rest = elem_z(wing_1, "outer_rail")
    with ctx.pose({wing_1_joint: 1.20}):
        wing_1_folded = elem_z(wing_1, "outer_rail")
    ctx.check(
        "rear wing folds upward",
        wing_1_rest is not None and wing_1_folded is not None and wing_1_folded > wing_1_rest + 0.30,
        details=f"rest={wing_1_rest}, folded={wing_1_folded}",
    )

    lower_rest = elem_z(lower, "outer_rail")
    with ctx.pose({lower_joint: 1.00}):
        lower_folded = elem_z(lower, "outer_rail")
    ctx.check(
        "lower frame folds upward",
        lower_rest is not None and lower_folded is not None and lower_folded > lower_rest + 0.25,
        details=f"rest={lower_rest}, folded={lower_folded}",
    )

    return ctx.report()


object_model = build_object_model()
