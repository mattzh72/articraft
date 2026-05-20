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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_one_hole_punch")

    body_finish = model.material("body_finish", rgba=(0.29, 0.31, 0.34, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.09, 0.10, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.50, 0.53, 0.56, 1.0))

    body = model.part("body")

    base_shell = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.110, 0.040, 0.008), 0.018),
        "base_shell",
    )
    body.visual(
        base_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=body_finish,
        name="base_shell",
    )

    pedestal = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.062, 0.032, 0.006), 0.014),
        "pedestal",
    )
    body.visual(
        pedestal,
        origin=Origin(xyz=(0.008, 0.0, 0.025)),
        material=body_finish,
        name="pedestal",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.015, 0.0, 0.035)),
        material=steel,
        name="die_ring",
    )
    body.visual(
        Box((0.030, 0.004, 0.022)),
        origin=Origin(xyz=(-0.020, -0.013, 0.029)),
        material=body_finish,
        name="rear_cheek_0",
    )
    body.visual(
        Box((0.030, 0.004, 0.022)),
        origin=Origin(xyz=(-0.020, 0.013, 0.029)),
        material=body_finish,
        name="rear_cheek_1",
    )
    body.visual(
        Box((0.012, 0.022, 0.004)),
        origin=Origin(xyz=(-0.039, 0.0, 0.033)),
        material=dark_steel,
        name="rear_bridge",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.110, 0.040, 0.046)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    handle = model.part("handle")
    handle_shell = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.084, 0.028, 0.006), 0.010),
        "handle_shell",
    )
    handle.visual(
        handle_shell,
        origin=Origin(xyz=(0.042, 0.0, 0.007)),
        material=handle_finish,
        name="handle_shell",
    )
    handle.visual(
        Box((0.018, 0.028, 0.004)),
        origin=Origin(xyz=(0.075, 0.0, 0.011)),
        material=handle_finish,
        name="front_pad",
    )
    handle.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_tube",
    )
    handle.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.050, 0.0, 0.0005)),
        material=steel,
        name="punch_stem",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.090, 0.028, 0.016)),
        mass=0.22,
        origin=Origin(xyz=(0.045, 0.0, 0.006)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.034, 0.0, 0.043)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    waste_flap = model.part("waste_flap")
    flap_panel = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.034, 0.028, 0.004), 0.0025),
        "flap_panel",
    )
    waste_flap.visual(
        flap_panel,
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material=body_finish,
        name="flap_panel",
    )
    waste_flap.visual(
        Cylinder(radius=0.0015, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="flap_hinge",
    )
    waste_flap.inertial = Inertial.from_geometry(
        Box((0.034, 0.028, 0.004)),
        mass=0.03,
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_waste_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=waste_flap,
        origin=Origin(xyz=(-0.002, 0.0, -0.0015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    waste_flap = object_model.get_part("waste_flap")
    handle_hinge = object_model.get_articulation("body_to_handle")
    flap_hinge = object_model.get_articulation("body_to_waste_flap")

    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="punch_stem",
        outer_elem="die_ring",
        margin=0.001,
        name="punch_stem stays centered over die",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="punch_stem",
        negative_elem="die_ring",
        min_gap=0.0,
        max_gap=0.004,
        name="closed punch stem sits just above die",
    )
    ctx.expect_gap(
        body,
        waste_flap,
        axis="z",
        positive_elem="base_shell",
        negative_elem="flap_panel",
        min_gap=0.0002,
        max_gap=0.004,
        name="waste flap hangs just beneath base",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(handle, elem="front_pad")
    rest_flap_aabb = ctx.part_element_world_aabb(waste_flap, elem="flap_panel")

    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        open_pad_aabb = ctx.part_element_world_aabb(handle, elem="front_pad")
        ctx.check(
            "handle lifts upward when opened",
            rest_pad_aabb is not None
            and open_pad_aabb is not None
            and (open_pad_aabb[0][2] + open_pad_aabb[1][2]) * 0.5
            > (rest_pad_aabb[0][2] + rest_pad_aabb[1][2]) * 0.5 + 0.025,
            details=f"rest={rest_pad_aabb}, open={open_pad_aabb}",
        )

    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_flap_aabb = ctx.part_element_world_aabb(waste_flap, elem="flap_panel")
        ctx.check(
            "waste flap swings downward",
            rest_flap_aabb is not None
            and open_flap_aabb is not None
            and (open_flap_aabb[0][2] + open_flap_aabb[1][2]) * 0.5
            < (rest_flap_aabb[0][2] + rest_flap_aabb[1][2]) * 0.5 - 0.008,
            details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
