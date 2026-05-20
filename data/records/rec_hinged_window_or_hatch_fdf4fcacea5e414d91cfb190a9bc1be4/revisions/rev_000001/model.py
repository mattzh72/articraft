from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gullwing_service_hatch")

    body_paint = model.material("deep_vehicle_blue", rgba=(0.04, 0.14, 0.26, 1.0))
    trim_rubber = model.material("black_epdm", rgba=(0.01, 0.012, 0.012, 1.0))
    dark_cavity = model.material("shadowed_service_bay", rgba=(0.015, 0.018, 0.020, 1.0))
    satin_steel = model.material("satin_stainless", rgba=(0.62, 0.64, 0.62, 1.0))
    latch_black = model.material("blackened_latch_steel", rgba=(0.02, 0.021, 0.022, 1.0))

    panel_w = 1.65
    panel_h = 1.05
    opening_w = 1.02
    opening_h = 0.60
    lid_w = 1.10
    lid_h = 0.68
    hinge_y = 0.060
    hinge_z = 0.360

    panel_profile = rounded_rect_profile(panel_w, panel_h, 0.095, corner_segments=10)
    opening_profile = rounded_rect_profile(opening_w, opening_h, 0.055, corner_segments=10)
    body_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(panel_profile, (opening_profile,), 0.045),
        "body_side_panel",
    )
    fixed_bezel_mesh = mesh_from_geometry(
        BezelGeometry(
            (opening_w, opening_h),
            (1.18, 0.74),
            0.030,
            opening_corner_radius=0.055,
            outer_corner_radius=0.075,
        ),
        "fixed_perimeter_bezel",
    )

    body = model.part("body")
    body.visual(
        body_panel_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_paint,
        name="side_skin",
    )
    body.visual(
        fixed_bezel_mesh,
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_paint,
        name="perimeter_frame",
    )
    body.visual(
        Box((opening_w, 0.012, opening_h)),
        origin=Origin(xyz=(0.0, -0.065, 0.0)),
        material=dark_cavity,
        name="recess_back",
    )
    body.visual(
        Box((opening_w, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, -0.030, opening_h / 2.0 - 0.012)),
        material=dark_cavity,
        name="recess_top_wall",
    )
    body.visual(
        Box((opening_w, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, -0.030, -opening_h / 2.0 + 0.012)),
        material=dark_cavity,
        name="recess_bottom_wall",
    )
    for x, name in ((-opening_w / 2.0 + 0.012, "recess_wall_0"), (opening_w / 2.0 - 0.012, "recess_wall_1")):
        body.visual(
            Box((0.035, 0.080, opening_h)),
            origin=Origin(xyz=(x, -0.030, 0.0)),
            material=dark_cavity,
            name=name,
        )

    body.visual(
        Box((1.16, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, 0.038, hinge_z - 0.010)),
        material=satin_steel,
        name="fixed_hinge_leaf",
    )
    for x, name in ((-0.46, "fixed_knuckle_0"), (0.0, "fixed_knuckle_1"), (0.46, "fixed_knuckle_2")):
        body.visual(
            Cylinder(radius=0.018, length=0.190),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=name,
        )

    for x, name in ((-0.34, "striker_0"), (0.34, "striker_1")):
        body.visual(
            Box((0.150, 0.018, 0.040)),
            origin=Origin(xyz=(x, 0.048, -opening_h / 2.0 - 0.012)),
            material=satin_steel,
            name=name,
        )
        body.visual(
            Box((0.088, 0.020, 0.012)),
            origin=Origin(xyz=(x, 0.060, -opening_h / 2.0 + 0.004)),
            material=dark_cavity,
            name=f"{name}_slot",
        )

    lid_profile = rounded_rect_profile(lid_w, lid_h, 0.065, corner_segments=10)
    lid_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(lid_profile, 0.030),
        "rigid_lid_shell",
    )

    lid = model.part("lid")
    lid.visual(
        lid_shell_mesh,
        origin=Origin(xyz=(0.0, 0.037, -lid_h / 2.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_paint,
        name="outer_panel",
    )
    lid.visual(
        Box((lid_w - 0.13, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.020, -0.055)),
        material=body_paint,
        name="upper_stiffener",
    )
    lid.visual(
        Box((lid_w - 0.13, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.020, -lid_h + 0.055)),
        material=body_paint,
        name="lower_stiffener",
    )
    for x, name in ((-lid_w / 2.0 + 0.055, "side_stiffener_0"), (lid_w / 2.0 - 0.055, "side_stiffener_1")):
        lid.visual(
            Box((0.026, 0.010, lid_h - 0.16)),
            origin=Origin(xyz=(x, 0.020, -lid_h / 2.0)),
            material=body_paint,
            name=name,
        )
    lid.visual(
        Box((lid_w - 0.18, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, 0.020, -0.075)),
        material=trim_rubber,
        name="upper_gasket",
    )
    lid.visual(
        Box((lid_w - 0.18, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, 0.020, -lid_h + 0.075)),
        material=trim_rubber,
        name="lower_gasket",
    )
    for x, name in ((-lid_w / 2.0 + 0.075, "side_gasket_0"), (lid_w / 2.0 - 0.075, "side_gasket_1")):
        lid.visual(
            Box((0.026, 0.008, lid_h - 0.22)),
            origin=Origin(xyz=(x, 0.020, -lid_h / 2.0)),
            material=trim_rubber,
            name=name,
        )

    lid.visual(
        Box((1.08, 0.008, 0.035)),
        origin=Origin(xyz=(0.0, 0.023, -0.018)),
        material=satin_steel,
        name="moving_hinge_leaf",
    )
    for x, name in ((-0.23, "lid_knuckle_0"), (0.23, "lid_knuckle_1")):
        lid.visual(
            Cylinder(radius=0.018, length=0.190),
            origin=Origin(xyz=(x, 0.003, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=name,
        )

    for x, name in ((-0.34, "latch_boss_0"), (0.34, "latch_boss_1")):
        lid.visual(
            Cylinder(radius=0.032, length=0.010),
            origin=Origin(xyz=(x, 0.020, -lid_h + 0.035), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=name,
        )

    hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.20),
    )

    del hinge  # keep the semantic name above; tests retrieve it from the model

    dog_specs = (
        ("latch_dog_0", -0.34, 1.0, -1.25, 0.0),
        ("latch_dog_1", 0.34, -1.0, 0.0, 1.25),
    )
    for dog_name, x, direction, lower, upper in dog_specs:
        dog = model.part(dog_name)
        dog.visual(
            Cylinder(radius=0.024, length=0.016),
            origin=Origin(xyz=(0.0, 0.033, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=latch_black,
            name="pivot_hub",
        )
        dog.visual(
            Box((0.098, 0.014, 0.030)),
            origin=Origin(xyz=(direction * 0.048, 0.033, 0.0)),
            material=latch_black,
            name="dog_bar",
        )
        dog.visual(
            Cylinder(radius=0.016, length=0.016),
            origin=Origin(xyz=(direction * 0.095, 0.033, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name="roller_nose",
        )
        model.articulation(
            f"lid_to_{dog_name}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=dog,
            origin=Origin(xyz=(x, 0.0, -lid_h + 0.035)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=lower, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dog_0 = object_model.get_part("latch_dog_0")
    dog_1 = object_model.get_part("latch_dog_1")
    hinge = object_model.get_articulation("body_to_lid")
    latch_0 = object_model.get_articulation("lid_to_latch_dog_0")
    latch_1 = object_model.get_articulation("lid_to_latch_dog_1")

    def element_center(part, elem):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    def part_bounds_center(part):
        bounds = ctx.part_world_aabb(part)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        lid,
        body,
        axis="y",
        min_gap=0.004,
        positive_elem="outer_panel",
        negative_elem="perimeter_frame",
        name="closed lid sits proud of the fixed perimeter",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xz",
        min_overlap=0.50,
        elem_a="outer_panel",
        elem_b="perimeter_frame",
        name="closed lid covers the fixed opening",
    )
    ctx.expect_overlap(
        dog_0,
        body,
        axes="x",
        min_overlap=0.040,
        elem_a="dog_bar",
        elem_b="striker_0",
        name="first latch dog aligns with its lower striker",
    )
    ctx.expect_overlap(
        dog_1,
        body,
        axes="x",
        min_overlap=0.040,
        elem_a="dog_bar",
        elem_b="striker_1",
        name="second latch dog aligns with its lower striker",
    )

    closed_lower_edge = element_center(lid, "lower_stiffener")
    closed_dog_0 = part_bounds_center(dog_0)
    closed_dog_1 = part_bounds_center(dog_1)

    with ctx.pose({hinge: 1.0}):
        raised_lower_edge = element_center(lid, "lower_stiffener")
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            min_gap=0.20,
            positive_elem="lower_stiffener",
            negative_elem="perimeter_frame",
            name="gullwing lid swings outward from the side",
        )

    with ctx.pose({latch_0: -1.0, latch_1: 1.0}):
        rotated_dog_0 = part_bounds_center(dog_0)
        rotated_dog_1 = part_bounds_center(dog_1)

    ctx.check(
        "upper hinge raises the lid away from the body",
        closed_lower_edge is not None
        and raised_lower_edge is not None
        and raised_lower_edge[1] > closed_lower_edge[1] + 0.25,
        details=f"closed={closed_lower_edge}, raised={raised_lower_edge}",
    )
    ctx.check(
        "latch dogs rotate on their local pivots",
        closed_dog_0 is not None
        and rotated_dog_0 is not None
        and closed_dog_1 is not None
        and rotated_dog_1 is not None
        and abs(rotated_dog_0[2] - closed_dog_0[2]) > 0.025
        and abs(rotated_dog_1[2] - closed_dog_1[2]) > 0.025,
        details=f"dog0={closed_dog_0}->{rotated_dog_0}, dog1={closed_dog_1}->{rotated_dog_1}",
    )

    return ctx.report()


object_model = build_object_model()
