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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    enamel_black = model.material("enamel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.30, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.22, 0.26, 0.40))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.10, 1.0))

    body = model.part("oven_body")
    body.visual(
        Box((0.460, 0.360, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=enamel_black,
        name="bottom_plate",
    )
    body.visual(
        Box((0.012, 0.360, 0.288)),
        origin=Origin(xyz=(-0.224, 0.000, 0.156)),
        material=dark_steel,
        name="left_wall",
    )
    body.visual(
        Box((0.012, 0.360, 0.288)),
        origin=Origin(xyz=(0.224, 0.000, 0.156)),
        material=dark_steel,
        name="right_wall",
    )
    body.visual(
        Box((0.460, 0.012, 0.288)),
        origin=Origin(xyz=(0.000, 0.174, 0.156)),
        material=dark_steel,
        name="back_wall",
    )
    body.visual(
        Box((0.460, 0.360, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.294)),
        material=dark_steel,
        name="top_shell",
    )
    body.visual(
        Box((0.108, 0.020, 0.282)),
        origin=Origin(xyz=(0.164, -0.170, 0.153)),
        material=brushed_steel,
        name="control_fascia",
    )
    body.visual(
        Box((0.076, 0.004, 0.020)),
        origin=Origin(xyz=(0.164, -0.178, 0.268)),
        material=dark_steel,
        name="brand_badge",
    )
    body.visual(
        Box((0.016, 0.004, 0.016)),
        origin=Origin(xyz=(0.198, -0.178, 0.052)),
        material=smoked_glass,
        name="indicator_lamp",
    )
    body.visual(
        Box((0.012, 0.328, 0.236)),
        origin=Origin(xyz=(0.110, 0.008, 0.130)),
        material=dark_steel,
        name="control_partition",
    )
    body.visual(
        Box((0.310, 0.024, 0.032)),
        origin=Origin(xyz=(-0.045, -0.168, 0.028)),
        material=brushed_steel,
        name="lower_sill",
    )
    body.visual(
        Box((0.310, 0.024, 0.030)),
        origin=Origin(xyz=(-0.045, -0.168, 0.255)),
        material=brushed_steel,
        name="upper_lintel",
    )
    body.visual(
        Box((0.020, 0.050, 0.045)),
        origin=Origin(xyz=(-0.194, -0.155, 0.0225)),
        material=brushed_steel,
        name="left_door_cheek",
    )
    body.visual(
        Box((0.020, 0.050, 0.045)),
        origin=Origin(xyz=(0.104, -0.155, 0.0225)),
        material=brushed_steel,
        name="right_door_cheek",
    )
    body.visual(
        Box((0.460, 0.022, 0.020)),
        origin=Origin(xyz=(0.000, -0.169, 0.304)),
        material=brushed_steel,
        name="top_front_trim",
    )
    body.visual(
        Box((0.050, 0.020, 0.020)),
        origin=Origin(xyz=(-0.180, -0.165, 0.304)),
        material=brushed_steel,
        name="top_left_return",
    )
    body.visual(
        Box((0.050, 0.020, 0.020)),
        origin=Origin(xyz=(0.100, -0.165, 0.304)),
        material=brushed_steel,
        name="top_right_return",
    )
    for foot_index, foot_x in enumerate((-0.170, 0.170)):
        for back_index, foot_y in enumerate((-0.135, 0.135)):
            body.visual(
                Box((0.050, 0.035, 0.010)),
                origin=Origin(xyz=(foot_x, foot_y, -0.005)),
                material=knob_black,
                name=f"foot_{foot_index}_{back_index}",
            )
    knob_z_positions = (0.225, 0.162, 0.099)
    for knob_index, knob_z in enumerate(knob_z_positions):
        body.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(
                xyz=(0.164, -0.182, knob_z),
                rpy=(math.pi / 2.0, 0.000, 0.000),
            ),
            material=dark_steel,
            name=f"knob_bushing_{knob_index}",
        )
        body.visual(
            Box((0.050, 0.002, 0.012)),
            origin=Origin(xyz=(0.164, -0.181, knob_z + 0.038)),
            material=dark_steel,
            name=f"knob_label_top_{knob_index}",
        )
        body.visual(
            Box((0.050, 0.002, 0.012)),
            origin=Origin(xyz=(0.164, -0.181, knob_z - 0.038)),
            material=dark_steel,
            name=f"knob_label_bottom_{knob_index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.460, 0.360, 0.310)),
        mass=6.8,
        origin=Origin(xyz=(0.000, 0.000, 0.155)),
    )

    for knob_index, knob_z in enumerate(knob_z_positions):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            Cylinder(radius=0.0055, length=0.012),
            origin=Origin(
                xyz=(0.000, -0.006, 0.000),
                rpy=(math.pi / 2.0, 0.000, 0.000),
            ),
            material=dark_steel,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(
                xyz=(0.000, -0.015, 0.000),
                rpy=(math.pi / 2.0, 0.000, 0.000),
            ),
            material=dark_steel,
            name="collar",
        )
        knob.visual(
            Cylinder(radius=0.022, length=0.022),
            origin=Origin(
                xyz=(0.000, -0.029, 0.000),
                rpy=(math.pi / 2.0, 0.000, 0.000),
            ),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.004, 0.014)),
            origin=Origin(xyz=(0.000, -0.042, 0.012)),
            material=brushed_steel,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.044, 0.048, 0.044)),
            mass=0.060,
            origin=Origin(xyz=(0.000, -0.026, 0.000)),
        )
        model.articulation(
            f"body_to_knob_{knob_index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.164, -0.184, knob_z)),
            axis=(0.000, -1.000, 0.000),
            motion_limits=MotionLimits(
                effort=0.35,
                velocity=5.0,
                lower=-2.35,
                upper=2.35,
            ),
        )

    door = model.part("door")
    door.visual(
        Box((0.300, 0.010, 0.020)),
        origin=Origin(xyz=(0.000, 0.005, 0.010)),
        material=brushed_steel,
        name="bottom_rail",
    )
    door.visual(
        Box((0.300, 0.010, 0.022)),
        origin=Origin(xyz=(0.000, 0.005, 0.199)),
        material=brushed_steel,
        name="top_rail",
    )
    door.visual(
        Box((0.026, 0.010, 0.168)),
        origin=Origin(xyz=(-0.137, 0.005, 0.104)),
        material=brushed_steel,
        name="left_stile",
    )
    door.visual(
        Box((0.026, 0.010, 0.168)),
        origin=Origin(xyz=(0.137, 0.005, 0.104)),
        material=brushed_steel,
        name="right_stile",
    )
    door.visual(
        Box((0.248, 0.004, 0.168)),
        origin=Origin(xyz=(0.000, 0.008, 0.104)),
        material=smoked_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.012, 0.034, 0.018)),
        origin=Origin(xyz=(-0.092, -0.011, 0.151)),
        material=brushed_steel,
        name="left_handle_post",
    )
    door.visual(
        Box((0.012, 0.034, 0.018)),
        origin=Origin(xyz=(0.092, -0.011, 0.151)),
        material=brushed_steel,
        name="right_handle_post",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.190),
        origin=Origin(
            xyz=(0.000, -0.021, 0.151),
            rpy=(0.000, math.pi / 2.0, 0.000),
        ),
        material=brushed_steel,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.300, 0.050, 0.210)),
        mass=1.4,
        origin=Origin(xyz=(0.000, -0.010, 0.105)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.045, -0.190, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="lower_sill",
        negative_elem="bottom_rail",
        max_penetration=0.001,
        max_gap=0.012,
        name="door closes just in front of the lower sill",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="x",
        elem_a="lower_sill",
        elem_b="bottom_rail",
        min_overlap=0.260,
        name="door spans the oven opening width",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: math.radians(82.0)}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens downward from the front hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.10
        and open_aabb[1][2] < closed_aabb[1][2] - 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    def _aabb_center(aabb):
        return (
            (
                (aabb[0][0] + aabb[1][0]) * 0.5,
                (aabb[0][1] + aabb[1][1]) * 0.5,
                (aabb[0][2] + aabb[1][2]) * 0.5,
            )
            if aabb is not None
            else None
        )

    for knob_index in range(3):
        knob = object_model.get_part(f"knob_{knob_index}")
        knob_joint = object_model.get_articulation(f"body_to_knob_{knob_index}")
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            positive_elem=f"knob_bushing_{knob_index}",
            negative_elem="shaft",
            max_penetration=0.0005,
            max_gap=0.001,
            name=f"knob {knob_index} is shaft-mounted to the fascia",
        )
        ctx.expect_overlap(
            body,
            knob,
            axes="xz",
            elem_a=f"knob_bushing_{knob_index}",
            elem_b="shaft",
            min_overlap=0.009,
            name=f"knob {knob_index} shaft stays centered on its bushing",
        )
        if knob_index == 0:
            rest_pointer = ctx.part_element_world_aabb(knob, elem="pointer")
            with ctx.pose({knob_joint: 1.3}):
                turned_pointer = ctx.part_element_world_aabb(knob, elem="pointer")
            rest_center = _aabb_center(rest_pointer)
            turned_center = _aabb_center(turned_pointer)
            ctx.check(
                "upper knob visibly rotates its indicator pointer",
                rest_center is not None
                and turned_center is not None
                and abs(turned_center[0] - rest_center[0]) > 0.008
                and abs(turned_center[2] - rest_center[2]) > 0.002,
                details=f"rest={rest_center}, turned={turned_center}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
