from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BOWL_XS = (-0.13, 0.13)
BOWL_CENTER_Z = 0.45
BOWL_BOTTOM_Z = BOWL_CENTER_Z - 0.095


def _bowl_shell_mesh(name: str):
    # A thin transparent lathed shell: narrow bottom neck, round globe belly,
    # and a smaller open refill mouth at the top.
    outer = [
        (0.044, -0.095),
        (0.078, -0.078),
        (0.105, -0.035),
        (0.113, 0.000),
        (0.103, 0.045),
        (0.082, 0.078),
        (0.058, 0.095),
    ]
    wall = 0.0035
    inner = [(max(0.0, r - wall), z) for r, z in outer]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.062,
            0.026,
            body_style="lobed",
            base_diameter=0.050,
            top_diameter=0.058,
            crown_radius=0.002,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=12, depth=0.0020, width=0.006),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_globe_candy_vendor")

    chrome = model.material("polished_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_metal = model.material("shadowed_metal", rgba=(0.12, 0.12, 0.13, 1.0))
    clear = model.material("clear_acrylic", rgba=(0.76, 0.93, 1.0, 0.30))
    red = model.material("red_enamel", rgba=(0.75, 0.05, 0.035, 1.0))
    amber = model.material("warm_candy_mix", rgba=(0.95, 0.43, 0.12, 0.85))
    candy_red = model.material("candy_red", rgba=(0.90, 0.04, 0.06, 1.0))
    candy_yellow = model.material("candy_yellow", rgba=(1.0, 0.86, 0.10, 1.0))
    candy_green = model.material("candy_green", rgba=(0.08, 0.65, 0.18, 1.0))
    candy_blue = model.material("candy_blue", rgba=(0.05, 0.22, 0.85, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.44, 0.24, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=chrome,
        name="countertop_plinth",
    )
    base.visual(
        Box((0.33, 0.16, 0.150)),
        origin=Origin(xyz=(0.0, -0.005, 0.145)),
        material=red,
        name="upper_body",
    )
    base.visual(
        Box((0.31, 0.010, 0.095)),
        origin=Origin(xyz=(0.0, -0.090, 0.170)),
        material=chrome,
        name="front_panel",
    )

    for idx, x in enumerate(BOWL_XS):
        base.visual(
            Cylinder(radius=0.044, length=0.118),
            origin=Origin(xyz=(x, 0.005, 0.279), rpy=(0.0, 0.0, 0.0)),
            material=chrome,
            name=f"neck_support_{idx}",
        )
        base.visual(
            Cylinder(radius=0.060, length=0.018),
            origin=Origin(xyz=(x, 0.005, 0.346), rpy=(0.0, 0.0, 0.0)),
            material=chrome,
            name=f"bowl_collar_{idx}",
        )
        base.visual(
            Cylinder(radius=0.043, length=0.012),
            origin=Origin(xyz=(x, -0.091, 0.205), rpy=(pi / 2, 0.0, 0.0)),
            material=chrome,
            name=f"knob_boss_{idx}",
        )
        base.visual(
            Box((0.070, 0.014, 0.030)),
            origin=Origin(xyz=(x, -0.083, 0.122)),
            material=dark_metal,
            name=f"chute_shadow_{idx}",
        )

    collection_cup = model.part("collection_cup")
    collection_cup.visual(
        Box((0.220, 0.110, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=chrome,
        name="cup_floor",
    )
    collection_cup.visual(
        Box((0.220, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, 0.044, 0.041)),
        material=chrome,
        name="rear_wall",
    )
    collection_cup.visual(
        Box((0.012, 0.100, 0.070)),
        origin=Origin(xyz=(-0.104, 0.0, 0.041)),
        material=chrome,
        name="side_wall_0",
    )
    collection_cup.visual(
        Box((0.012, 0.100, 0.070)),
        origin=Origin(xyz=(0.104, 0.0, 0.041)),
        material=chrome,
        name="side_wall_1",
    )
    collection_cup.visual(
        Box((0.180, 0.008, 0.040)),
        origin=Origin(xyz=(0.0, 0.046, 0.048)),
        material=chrome,
        name="mount_flange",
    )
    collection_cup.visual(
        Box((0.180, 0.035, 0.010)),
        origin=Origin(xyz=(0.0, -0.030, 0.017)),
        material=amber,
        name="shared_candy_lip",
    )
    model.articulation(
        "base_to_collection_cup",
        ArticulationType.FIXED,
        parent=base,
        child=collection_cup,
        origin=Origin(xyz=(0.0, -0.145, 0.075)),
    )

    for idx, x in enumerate(BOWL_XS):
        bowl = model.part(f"bowl_{idx}")
        bowl.visual(
            _bowl_shell_mesh(f"globe_shell_{idx}"),
            material=clear,
            name="clear_globe",
        )
        bowl.visual(
            Cylinder(radius=0.050, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.083)),
            material=amber,
            name="candy_mound",
        )
        for candy_idx, (cx, cy, mat) in enumerate(
            (
                (-0.028, -0.010, candy_red),
                (-0.006, 0.018, candy_yellow),
                (0.022, -0.017, candy_green),
                (0.035, 0.014, candy_blue),
            )
        ):
            bowl.visual(
                Sphere(radius=0.012),
                origin=Origin(xyz=(cx, cy, -0.067)),
                material=mat,
                name=f"candy_{candy_idx}",
            )
        bowl.visual(
            Box((0.070, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.065, 0.093)),
            material=chrome,
            name="rear_hinge_leaf",
        )
        model.articulation(
            f"base_to_bowl_{idx}",
            ArticulationType.FIXED,
            parent=base,
            child=bowl,
            origin=Origin(xyz=(x, 0.005, BOWL_CENTER_Z)),
        )

        knob = model.part(f"knob_{idx}")
        knob.visual(
            Cylinder(radius=0.011, length=0.040),
            origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=dark_metal,
            name="shaft",
        )
        knob.visual(
            _knob_mesh(f"lobed_dispense_knob_{idx}"),
            origin=Origin(xyz=(0.0, -0.053, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=chrome,
            name="grip_cap",
        )
        model.articulation(
            f"base_to_knob_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=knob,
            origin=Origin(xyz=(x, -0.097, 0.205)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=8.0),
        )

        lid = model.part(f"lid_{idx}")
        lid.visual(
            Cylinder(radius=0.060, length=0.010),
            origin=Origin(xyz=(0.0, -0.055, 0.006)),
            material=red,
            name="round_lid",
        )
        lid.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(0.0, -0.055, 0.018)),
            material=chrome,
            name="lid_pull",
        )
        lid.visual(
            Box((0.048, 0.014, 0.006)),
            origin=Origin(xyz=(0.0, -0.007, 0.006)),
            material=chrome,
            name="hinge_tab",
        )
        lid.visual(
            Cylinder(radius=0.005, length=0.064),
            origin=Origin(xyz=(0.0, 0.001, 0.006), rpy=(0.0, pi / 2, 0.0)),
            material=chrome,
            name="hinge_barrel",
        )
        model.articulation(
            f"bowl_{idx}_to_lid",
            ArticulationType.REVOLUTE,
            parent=bowl,
            child=lid,
            origin=Origin(xyz=(0.0, 0.055, 0.095)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=1.25),
        )

    collection_door = model.part("collection_door")
    collection_door.visual(
        Box((0.165, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, -0.004, 0.034)),
        material=clear,
        name="clear_flap",
    )
    collection_door.visual(
        Cylinder(radius=0.006, length=0.178),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(0.0, pi / 2, 0.0)),
        material=chrome,
        name="front_hinge_barrel",
    )
    collection_door.visual(
        Box((0.170, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.002, 0.000)),
        material=chrome,
        name="front_hinge_leaf",
    )
    collection_door.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, -0.008, 0.046), rpy=(pi / 2, 0.0, 0.0)),
        material=chrome,
        name="flap_pull",
    )
    model.articulation(
        "collection_cup_to_door",
        ArticulationType.REVOLUTE,
        parent=collection_cup,
        child=collection_door,
        origin=Origin(xyz=(0.0, -0.061, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    cup = object_model.get_part("collection_cup")
    door = object_model.get_part("collection_door")
    door_joint = object_model.get_articulation("collection_cup_to_door")

    ctx.expect_gap(
        base,
        cup,
        axis="y",
        positive_elem="front_panel",
        negative_elem="mount_flange",
        max_gap=0.002,
        max_penetration=0.00001,
        name="shared collection cup is mounted to the front body",
    )
    ctx.expect_overlap(
        cup,
        base,
        axes="x",
        elem_a="mount_flange",
        elem_b="front_panel",
        min_overlap=0.12,
        name="collection cup flange spans the shared vendor center",
    )

    for idx in range(2):
        bowl = object_model.get_part(f"bowl_{idx}")
        knob = object_model.get_part(f"knob_{idx}")
        lid = object_model.get_part(f"lid_{idx}")
        knob_joint = object_model.get_articulation(f"base_to_knob_{idx}")
        lid_joint = object_model.get_articulation(f"bowl_{idx}_to_lid")

        ctx.check(
            f"knob {idx} uses continuous rotary dispense motion",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=str(knob_joint.articulation_type),
        )
        ctx.expect_gap(
            bowl,
            base,
            axis="z",
            positive_elem="clear_globe",
            negative_elem=f"bowl_collar_{idx}",
            max_gap=0.004,
            max_penetration=0.001,
            name=f"globe {idx} seats on its metal collar",
        )
        ctx.expect_gap(
            base,
            knob,
            axis="y",
            positive_elem=f"knob_boss_{idx}",
            negative_elem="shaft",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"knob {idx} shaft meets its front boss",
        )

        closed_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_joint: 0.95}):
            open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            f"refill lid {idx} opens upward on rear hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.025,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    closed_door = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 0.85}):
        open_door = ctx.part_world_aabb(door)
    ctx.check(
        "collection door swings forward from the shared cup",
        closed_door is not None
        and open_door is not None
        and open_door[0][1] < closed_door[0][1] - 0.010,
        details=f"closed={closed_door}, open={open_door}",
    )

    return ctx.report()


object_model = build_object_model()
