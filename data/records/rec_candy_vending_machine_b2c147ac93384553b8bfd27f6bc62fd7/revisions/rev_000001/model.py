from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_column_candy_vender")

    enamel = model.material("deep_red_enamel", rgba=(0.72, 0.04, 0.025, 1.0))
    dark_enamel = model.material("dark_red_shadow", rgba=(0.30, 0.015, 0.012, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.80, 0.74, 1.0))
    black = model.material("black_chute", rgba=(0.015, 0.014, 0.012, 1.0))
    clear = model.material("clear_acrylic", rgba=(0.78, 0.93, 1.0, 0.34))
    glass_edge = model.material("blue_glass_edge", rgba=(0.52, 0.86, 1.0, 0.50))
    candy_a = model.material("candy_warm_mix", rgba=(1.0, 0.42, 0.12, 0.86))
    candy_b = model.material("candy_cool_mix", rgba=(0.28, 0.62, 1.0, 0.86))
    cream = model.material("cream_label", rgba=(0.93, 0.84, 0.64, 1.0))

    cabinet = model.part("cabinet")

    # Shared lower cabinet, plinth, top deck, and rear spine.  Dimensions are
    # roughly countertop vending-machine scale.
    cabinet.visual(
        Box((0.72, 0.34, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=enamel,
        name="cabinet_body",
    )
    cabinet.visual(
        Box((0.80, 0.40, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_enamel,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.78, 0.38, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=enamel,
        name="top_deck",
    )
    cabinet.visual(
        Box((0.68, 0.035, 0.62)),
        origin=Origin(xyz=(0.0, 0.172, 0.82)),
        material=enamel,
        name="rear_spine",
    )
    cabinet.visual(
        Box((0.66, 0.012, 0.35)),
        origin=Origin(xyz=(0.0, -0.176, 0.285)),
        material=cream,
        name="front_panel",
    )

    # Two clear cylindrical canisters on red collars.  The canister walls are
    # actual thin shells, not solid transparent placeholders.
    canister_wall = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.128, 0.000),
            (0.133, 0.025),
            (0.133, 0.400),
            (0.116, 0.470),
        ],
        inner_profile=[
            (0.116, 0.012),
            (0.121, 0.034),
            (0.121, 0.390),
            (0.104, 0.456),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )

    for i, x in enumerate((-0.205, 0.205)):
        cabinet.visual(
            Cylinder(radius=0.142, length=0.030),
            origin=Origin(xyz=(x, 0.0, 0.555)),
            material=enamel,
            name=f"canister_collar_{i}",
        )
        cabinet.visual(
            mesh_from_geometry(canister_wall, f"canister_wall_{i}"),
            origin=Origin(xyz=(x, 0.0, 0.570)),
            material=clear,
            name=f"clear_canister_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.108, length=0.275),
            origin=Origin(xyz=(x, 0.0, 0.707)),
            material=candy_a if i == 0 else candy_b,
            name=f"candy_fill_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.124, length=0.032),
            origin=Origin(xyz=(x, 0.0, 1.055)),
            material=enamel,
            name=f"canister_lid_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.044, length=0.018),
            origin=Origin(xyz=(x, 0.0, 1.080)),
            material=chrome,
            name=f"lid_knob_{i}",
        )

    # Front selector stations: collars, labels, coin slots, and a central chute.
    for i, x in enumerate((-0.205, 0.205)):
        cabinet.visual(
            Box((0.145, 0.010, 0.040)),
            origin=Origin(xyz=(x, -0.185, 0.410)),
            material=cream,
            name=f"selector_label_{i}",
        )
        cabinet.visual(
            Box((0.085, 0.012, 0.014)),
            origin=Origin(xyz=(x, -0.192, 0.382)),
            material=black,
            name=f"coin_slot_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.060, length=0.020),
            origin=Origin(xyz=(x, -0.185, 0.315), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"shaft_bearing_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.026, length=0.012),
            origin=Origin(xyz=(x, -0.181, 0.315), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"shaft_socket_{i}",
        )

    # Chute mouth trim and dark receiving pocket.
    cabinet.visual(
        Box((0.245, 0.030, 0.135)),
        origin=Origin(xyz=(0.0, -0.165, 0.205)),
        material=black,
        name="chute_pocket",
    )
    cabinet.visual(
        Box((0.285, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.192, 0.292)),
        material=chrome,
        name="chute_top_trim",
    )
    cabinet.visual(
        Box((0.285, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.192, 0.118)),
        material=chrome,
        name="chute_bottom_trim",
    )
    for x in (-0.142, 0.142):
        cabinet.visual(
            Box((0.020, 0.018, 0.174)),
            origin=Origin(xyz=(x, -0.192, 0.205)),
            material=chrome,
            name=f"chute_side_trim_{0 if x < 0 else 1}",
        )
    cabinet.visual(
        Box((0.205, 0.160, 0.012)),
        origin=Origin(xyz=(0.0, -0.120, 0.180), rpy=(0.28, 0.0, 0.0)),
        material=black,
        name="sloped_chute_floor",
    )
    for x in (-0.138, 0.138):
        cabinet.visual(
            Cylinder(radius=0.012, length=0.044),
            origin=Origin(xyz=(x, -0.204, 0.128), rpy=(0.0, pi / 2.0, 0.0)),
            material=chrome,
            name=f"flap_hinge_knuckle_{0 if x < 0 else 1}",
        )

    # Continuous selector knobs, each on its own front-to-back horizontal shaft.
    for i, x in enumerate((-0.205, 0.205)):
        knob = model.part(f"selector_knob_{i}")
        knob.visual(
            mesh_from_geometry(
                KnobGeometry(
                    0.110,
                    0.055,
                    body_style="lobed",
                    base_diameter=0.074,
                    top_diameter=0.100,
                    crown_radius=0.004,
                    grip=KnobGrip(style="ribbed", count=12, depth=0.0018),
                    indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
                    center=False,
                ),
                f"selector_knob_mesh_{i}",
            ),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.017, length=0.030),
            origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="horizontal_shaft",
        )
        model.articulation(
            f"cabinet_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=knob,
            origin=Origin(xyz=(x, -0.195, 0.315)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=8.0),
        )

    # Hinged collection flap.  Its part frame is the lower hinge axis; positive
    # rotation opens the top outward and downward around the small lower hinge.
    flap = model.part("collection_flap")
    flap.visual(
        Box((0.218, 0.010, 0.128)),
        origin=Origin(xyz=(0.0, -0.006, 0.064)),
        material=enamel,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.012, length=0.102),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.160, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.016, 0.072)),
        material=chrome,
        name="flap_pull_lip",
    )
    model.articulation(
        "cabinet_to_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(0.0, -0.198, 0.128)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knob_0 = object_model.get_part("selector_knob_0")
    knob_1 = object_model.get_part("selector_knob_1")
    flap = object_model.get_part("collection_flap")
    cabinet = object_model.get_part("cabinet")
    joint_0 = object_model.get_articulation("cabinet_to_knob_0")
    joint_1 = object_model.get_articulation("cabinet_to_knob_1")
    flap_joint = object_model.get_articulation("cabinet_to_flap")

    ctx.allow_overlap(
        cabinet,
        flap,
        elem_a="chute_bottom_trim",
        elem_b="flap_hinge_barrel",
        reason=(
            "The flap hinge barrel is intentionally seated in the lower "
            "chute trim as a captured hinge knuckle."
        ),
    )

    ctx.check(
        "two selector knobs are continuous",
        joint_0.articulation_type == ArticulationType.CONTINUOUS
        and joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={joint_0.articulation_type}, {joint_1.articulation_type}",
    )
    ctx.check(
        "selector shafts are horizontal front-to-back",
        abs(joint_0.axis[1]) > 0.99 and abs(joint_1.axis[1]) > 0.99,
        details=f"axes={joint_0.axis}, {joint_1.axis}",
    )
    ctx.expect_contact(
        knob_0,
        cabinet,
        elem_a="horizontal_shaft",
        elem_b="shaft_bearing_0",
        contact_tol=0.004,
        name="first knob shaft seats at its bearing",
    )
    ctx.expect_contact(
        knob_1,
        cabinet,
        elem_a="horizontal_shaft",
        elem_b="shaft_bearing_1",
        contact_tol=0.004,
        name="second knob shaft seats at its bearing",
    )
    ctx.expect_contact(
        flap,
        cabinet,
        elem_a="flap_hinge_barrel",
        elem_b="chute_bottom_trim",
        contact_tol=0.025,
        name="collection flap hinge is mounted at lower chute mouth",
    )

    closed_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 0.85}):
        open_aabb = ctx.part_world_aabb(flap)

    ctx.check(
        "collection flap opens outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.020,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.check(
        "collection flap rotates on a lower horizontal hinge",
        flap_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(flap_joint.axis[0]) > 0.99
        and flap_joint.motion_limits.lower == 0.0
        and flap_joint.motion_limits.upper >= 1.0,
        details=(
            f"type={flap_joint.articulation_type}, axis={flap_joint.axis}, "
            f"limits={flap_joint.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
