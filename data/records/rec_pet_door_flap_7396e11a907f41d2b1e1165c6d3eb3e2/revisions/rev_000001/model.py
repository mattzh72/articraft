from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panel_door_with_pet_flap")

    painted_wood = Material("warm_white_painted_wood", color=(0.86, 0.82, 0.72, 1.0))
    slightly_darker_trim = Material("cream_trim", color=(0.78, 0.74, 0.65, 1.0))
    smoked_plastic = Material("translucent_smoked_plastic", color=(0.12, 0.16, 0.17, 0.55))
    black_rubber = Material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    brushed_metal = Material("brushed_metal", color=(0.62, 0.60, 0.55, 1.0))

    door_w = 0.90
    door_h = 2.05
    door_t = 0.045

    opening_w = 0.38
    opening_z_min = 0.31
    opening_z_max = 0.79
    hinge_z = 0.75

    door = model.part("panel_door")
    # The human-sized door is fixed in this model.  Its lower opening is built
    # from real surrounding members rather than a solid slab behind the flap.
    side_w = (door_w - opening_w) / 2.0
    door.visual(
        Box((side_w, door_t, door_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + side_w / 2.0), 0.0, door_h / 2.0)),
        material=painted_wood,
        name="door_left_stile",
    )
    door.visual(
        Box((side_w, door_t, door_h)),
        origin=Origin(xyz=((opening_w / 2.0 + side_w / 2.0), 0.0, door_h / 2.0)),
        material=painted_wood,
        name="door_right_stile",
    )
    door.visual(
        Box((opening_w, door_t, opening_z_min)),
        origin=Origin(xyz=(0.0, 0.0, opening_z_min / 2.0)),
        material=painted_wood,
        name="lower_panel_rail",
    )
    door.visual(
        Box((opening_w, door_t, door_h - opening_z_max)),
        origin=Origin(xyz=(0.0, 0.0, (opening_z_max + door_h) / 2.0)),
        material=painted_wood,
        name="upper_panel_field",
    )

    # Raised moulding on the front of the fixed panel door makes it read as a
    # domestic side-hinged door, not merely a flat rectangular frame.
    front_y = door_t / 2.0 + 0.006
    upper_mould_z = 1.38
    upper_mould_h = 0.78
    upper_mould_w = 0.58
    mould_t = 0.012
    for x, name in (
        (-(upper_mould_w / 2.0 - mould_t / 2.0), "upper_mould_left"),
        ((upper_mould_w / 2.0 - mould_t / 2.0), "upper_mould_right"),
    ):
        door.visual(
            Box((mould_t, 0.012, upper_mould_h)),
            origin=Origin(xyz=(x, front_y, upper_mould_z)),
            material=slightly_darker_trim,
            name=name,
        )
    for z, name in (
        ((upper_mould_z + upper_mould_h / 2.0 - mould_t / 2.0), "upper_mould_top"),
        ((upper_mould_z - upper_mould_h / 2.0 + mould_t / 2.0), "upper_mould_bottom"),
    ):
        door.visual(
            Box((upper_mould_w, 0.012, mould_t)),
            origin=Origin(xyz=(0.0, front_y, z)),
            material=slightly_darker_trim,
            name=name,
        )

    pet_bezel = BezelGeometry(
        opening_size=(0.34, 0.45),
        outer_size=(0.50, 0.61),
        depth=0.014,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.025,
        outer_corner_radius=0.040,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.003),
    )
    door.visual(
        mesh_from_geometry(pet_bezel, "pet_trim"),
        origin=Origin(xyz=(0.0, door_t / 2.0 + 0.008, (opening_z_min + opening_z_max) / 2.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=slightly_darker_trim,
        name="pet_trim",
    )

    # Fixed side hinge hardware on the left edge marks the large door as a
    # side-hinged human door even though that outer door is not articulated here.
    for i, z in enumerate((0.36, 1.05, 1.74)):
        door.visual(
            Box((0.070, 0.010, 0.145)),
            origin=Origin(xyz=(-door_w / 2.0 + 0.030, door_t / 2.0 + 0.005, z)),
            material=brushed_metal,
            name=f"side_hinge_leaf_{i}",
        )
        door.visual(
            Cylinder(radius=0.014, length=0.160),
            origin=Origin(xyz=(-door_w / 2.0 - 0.008, 0.0, z)),
            material=brushed_metal,
            name=f"side_hinge_barrel_{i}",
        )

    # Parent-side top hinge knuckles and a strap, mounted just above the pet
    # opening.  The moving flap has a separate center barrel on the same axis.
    door.visual(
        Box((0.38, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, door_t / 2.0 + 0.005, hinge_z + 0.037)),
        material=brushed_metal,
        name="pet_hinge_strap",
    )
    for x, name in ((-0.170, "pet_hinge_knuckle_0"), (0.170, "pet_hinge_knuckle_1")):
        door.visual(
            Cylinder(radius=0.014, length=0.035),
            origin=Origin(xyz=(x, 0.0, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_metal,
            name=name,
        )
        door.visual(
            Box((0.035, 0.008, 0.052)),
            origin=Origin(xyz=(x, 0.010, hinge_z + 0.026)),
            material=brushed_metal,
            name=f"{name}_web",
        )

    pet_flap = model.part("pet_flap")
    flap_w = 0.300
    flap_h = 0.380
    flap_t = 0.012
    hinge_r = 0.014
    pet_flap.visual(
        Box((flap_w, flap_t, flap_h)),
        origin=Origin(xyz=(0.0, 0.0, -(hinge_r + flap_h / 2.0))),
        material=smoked_plastic,
        name="flap_panel",
    )
    pet_flap.visual(
        Cylinder(radius=hinge_r, length=0.305),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_metal,
        name="flap_hinge_barrel",
    )
    pet_flap.visual(
        Box((flap_w, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, flap_t / 2.0 + 0.003, -(hinge_r + flap_h - 0.009))),
        material=black_rubber,
        name="bottom_magnetic_strip",
    )
    for x, name in ((-(flap_w / 2.0 - 0.006), "flap_side_gasket_0"), ((flap_w / 2.0 - 0.006), "flap_side_gasket_1")):
        pet_flap.visual(
            Box((0.012, 0.006, flap_h)),
            origin=Origin(xyz=(x, flap_t / 2.0 + 0.003, -(hinge_r + flap_h / 2.0))),
            material=black_rubber,
            name=name,
        )

    model.articulation(
        "door_to_pet_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=pet_flap,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-1.15, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("panel_door")
    flap = object_model.get_part("pet_flap")
    hinge = object_model.get_articulation("door_to_pet_flap")

    ctx.check(
        "pet flap uses a bounded top hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (1.0, 0.0, 0.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower < 0.0
        and hinge.motion_limits.upper > 0.0,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    ctx.expect_contact(
        flap,
        door,
        elem_a="flap_hinge_barrel",
        elem_b="pet_hinge_knuckle_0",
        contact_tol=0.001,
        name="flap barrel is captured by one hinge knuckle",
    )
    ctx.expect_contact(
        flap,
        door,
        elem_a="flap_hinge_barrel",
        elem_b="pet_hinge_knuckle_1",
        contact_tol=0.001,
        name="flap barrel is captured by the opposite hinge knuckle",
    )
    ctx.expect_gap(
        flap,
        door,
        axis="z",
        positive_elem="flap_panel",
        negative_elem="lower_panel_rail",
        min_gap=0.020,
        name="closed flap clears lower door rail",
    )
    ctx.expect_gap(
        door,
        flap,
        axis="z",
        positive_elem="upper_panel_field",
        negative_elem="flap_panel",
        min_gap=0.020,
        name="closed flap clears upper door rail",
    )
    ctx.expect_gap(
        door,
        flap,
        axis="x",
        positive_elem="door_right_stile",
        negative_elem="flap_panel",
        min_gap=0.020,
        name="closed flap clears right side of opening",
    )
    ctx.expect_gap(
        flap,
        door,
        axis="x",
        positive_elem="flap_panel",
        negative_elem="door_left_stile",
        min_gap=0.020,
        name="closed flap clears left side of opening",
    )

    closed_bottom = ctx.part_element_world_aabb(flap, elem="bottom_magnetic_strip")
    closed_y = None if closed_bottom is None else (closed_bottom[0][1] + closed_bottom[1][1]) / 2.0
    with ctx.pose({hinge: 0.75}):
        open_bottom = ctx.part_element_world_aabb(flap, elem="bottom_magnetic_strip")
        open_y = None if open_bottom is None else (open_bottom[0][1] + open_bottom[1][1]) / 2.0
    ctx.check(
        "positive hinge swing moves pet flap outward",
        closed_y is not None and open_y is not None and open_y > closed_y + 0.15,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    return ctx.report()


object_model = build_object_model()
