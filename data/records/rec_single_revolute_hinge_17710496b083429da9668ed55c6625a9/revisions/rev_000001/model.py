from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_weld_on_hinge")

    dark_steel = model.material("dark_oiled_steel", color=(0.08, 0.085, 0.085, 1.0))
    worn_steel = model.material("worn_steel_edges", color=(0.42, 0.40, 0.36, 1.0))
    weld_blue = model.material("heat_tinted_welds", color=(0.16, 0.15, 0.17, 1.0))

    hinge_length = 0.320
    leaf_width = 0.140
    leaf_thickness = 0.010
    mount_thickness = 0.026
    barrel_outer_radius = 0.026
    bore_radius = 0.0120
    pin_radius = 0.0125
    knuckle_length = 0.055
    knuckle_gap = 0.006
    first_knuckle_z = -0.122
    pitch = knuckle_length + knuckle_gap

    knuckle_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(barrel_outer_radius, -knuckle_length / 2.0), (barrel_outer_radius, knuckle_length / 2.0)],
            [(bore_radius, -knuckle_length / 2.0), (bore_radius, knuckle_length / 2.0)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        "hollow_knuckle_sleeve",
    )

    ground_leaf = model.part("ground_leaf")
    ground_leaf.visual(
        Box((0.172, mount_thickness, 0.360)),
        origin=Origin(xyz=(-0.118, -0.020, 0.0)),
        material=dark_steel,
        name="mounting_plate",
    )
    ground_leaf.visual(
        Box((leaf_width, leaf_thickness, hinge_length)),
        origin=Origin(xyz=(-(barrel_outer_radius + leaf_width / 2.0), 0.0, 0.0)),
        material=dark_steel,
        name="ground_leaf_panel",
    )
    ground_leaf.visual(
        Cylinder(radius=pin_radius, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=worn_steel,
        name="pin_barrel",
    )
    ground_leaf.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=worn_steel,
        name="top_pin_head",
    )
    ground_leaf.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.179)),
        material=worn_steel,
        name="bottom_pin_rivet",
    )

    for index in (0, 2, 4):
        z = first_knuckle_z + pitch * index
        ground_leaf.visual(
            knuckle_mesh,
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_steel,
            name=f"ground_knuckle_{index // 2}",
        )
        ground_leaf.visual(
            Box((0.024, leaf_thickness + 0.004, knuckle_length)),
            origin=Origin(xyz=(-(barrel_outer_radius + 0.004), 0.0, z)),
            material=dark_steel,
            name=f"ground_knuckle_web_{index // 2}",
        )

    for x in (-0.198, -0.034):
        ground_leaf.visual(
            Cylinder(radius=0.0045, length=0.325),
            origin=Origin(xyz=(x, -0.004, 0.0)),
            material=weld_blue,
            name=f"weld_bead_{0 if x < -0.1 else 1}",
        )

    swing_leaf = model.part("swing_leaf")
    swing_leaf.visual(
        Box((leaf_width, leaf_thickness, hinge_length * 0.94)),
        origin=Origin(xyz=(barrel_outer_radius + leaf_width / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="swing_leaf_panel",
    )
    swing_leaf.visual(
        Box((0.115, 0.004, hinge_length * 0.78)),
        origin=Origin(xyz=(barrel_outer_radius + 0.073, 0.007, 0.0)),
        material=worn_steel,
        name="raised_wear_land",
    )

    for serial, index in enumerate((1, 3)):
        z = first_knuckle_z + pitch * index
        swing_leaf.visual(
            knuckle_mesh,
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_steel,
            name=f"swing_knuckle_{serial}",
        )
        swing_leaf.visual(
            Box((0.024, leaf_thickness + 0.004, knuckle_length)),
            origin=Origin(xyz=(barrel_outer_radius + 0.004, 0.0, z)),
            material=dark_steel,
            name=f"swing_knuckle_web_{serial}",
        )

    model.articulation(
        "pin_joint",
        ArticulationType.REVOLUTE,
        parent=ground_leaf,
        child=swing_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.2, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_leaf = object_model.get_part("ground_leaf")
    swing_leaf = object_model.get_part("swing_leaf")
    pin_joint = object_model.get_articulation("pin_joint")

    for knuckle_name in ("swing_knuckle_0", "swing_knuckle_1"):
        ctx.allow_overlap(
            ground_leaf,
            swing_leaf,
            elem_a="pin_barrel",
            elem_b=knuckle_name,
            reason="The steel pin is intentionally represented as a tiny interference fit inside the swinging knuckle bore.",
        )
        ctx.expect_within(
            ground_leaf,
            swing_leaf,
            axes="xy",
            inner_elem="pin_barrel",
            outer_elem=knuckle_name,
            margin=0.0,
            name=f"pin centered through {knuckle_name}",
        )
        ctx.expect_overlap(
            ground_leaf,
            swing_leaf,
            axes="z",
            elem_a="pin_barrel",
            elem_b=knuckle_name,
            min_overlap=0.050,
            name=f"pin retained through {knuckle_name}",
        )

    ctx.check(
        "single revolute pin joint",
        len(object_model.articulations) == 1 and pin_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "pin axis is hinge barrel",
        tuple(round(v, 6) for v in pin_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pin_joint.axis}",
    )
    ctx.expect_gap(
        swing_leaf,
        ground_leaf,
        axis="x",
        positive_elem="swing_leaf_panel",
        negative_elem="ground_leaf_panel",
        min_gap=0.045,
        name="rectangular leaves sit on opposite sides of barrel",
    )

    mount_aabb = ctx.part_element_world_aabb(ground_leaf, elem="mounting_plate")
    leaf_aabb = ctx.part_element_world_aabb(ground_leaf, elem="ground_leaf_panel")
    if mount_aabb is not None and leaf_aabb is not None:
        mount_thickness = mount_aabb[1][1] - mount_aabb[0][1]
        leaf_thickness = leaf_aabb[1][1] - leaf_aabb[0][1]
        ctx.check(
            "ground leaf has thicker weld plate",
            mount_thickness > leaf_thickness * 2.0,
            details=f"mount_thickness={mount_thickness}, leaf_thickness={leaf_thickness}",
        )
    else:
        ctx.fail("ground leaf has thicker weld plate", "missing mounting plate or ground leaf panel AABB")

    def _aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_aabb = ctx.part_element_world_aabb(swing_leaf, elem="swing_leaf_panel")
    with ctx.pose({pin_joint: 1.20}):
        turned_aabb = ctx.part_element_world_aabb(swing_leaf, elem="swing_leaf_panel")
    if rest_aabb is not None and turned_aabb is not None:
        rest_center = _aabb_center(rest_aabb)
        turned_center = _aabb_center(turned_aabb)
        ctx.check(
            "swing leaf rotates around vertical pin",
            turned_center[1] > rest_center[1] + 0.070 and turned_center[0] < rest_center[0] - 0.020,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )
    else:
        ctx.fail("swing leaf rotates around vertical pin", "missing swing leaf panel AABB")

    return ctx.report()


object_model = build_object_model()
