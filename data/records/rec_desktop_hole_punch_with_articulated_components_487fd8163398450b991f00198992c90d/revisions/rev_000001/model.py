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
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_hole_punch")

    body_metal = model.material("body_metal", rgba=(0.30, 0.33, 0.36, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.12, 0.13, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    bumper = model.material("bumper", rgba=(0.08, 0.08, 0.09, 1.0))
    stop_plastic = model.material("stop_plastic", rgba=(0.18, 0.20, 0.22, 1.0))

    def xz_section(width: float, height: float, radius: float, y: float, z_center: float):
        return [(x, y, z + z_center) for x, z in rounded_rect_profile(width, height, radius)]

    base = model.part("base")

    base_plate = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.290, 0.150, 0.022), 0.012),
        "base_plate",
    )
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_metal,
        name="base_plate",
    )

    pedestal_mesh = mesh_from_geometry(
        section_loft(
            [
                xz_section(0.110, 0.038, 0.012, -0.048, 0.031),
                xz_section(0.100, 0.048, 0.014, -0.006, 0.034),
                xz_section(0.078, 0.040, 0.012, 0.040, 0.028),
            ]
        ),
        "pedestal",
    )
    base.visual(pedestal_mesh, material=body_metal, name="pedestal")
    base.visual(
        Box((0.050, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.058, 0.025)),
        material=body_metal,
        name="die_bridge",
    )
    for x_pos in (-0.040, 0.040):
        base.visual(
            Cylinder(radius=0.007, length=0.010),
            origin=Origin(xyz=(x_pos, 0.058, 0.041)),
            material=steel,
            name=f"die_cap_{0 if x_pos < 0 else 1}",
        )
    base.visual(
        Box((0.230, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, 0.052, 0.025)),
        material=steel,
        name="paper_fence",
    )
    base.visual(
        Box((0.232, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.042, 0.035)),
        material=steel,
        name="guide_rail",
    )
    for x_pos in (-0.102, 0.0, 0.102):
        base.visual(
            Box((0.010, 0.010, 0.014)),
            origin=Origin(xyz=(x_pos, 0.047, 0.031)),
            material=steel,
            name=f"rail_bracket_{int((x_pos + 0.102) * 1000)}",
        )
    for x_pos in (-0.040, 0.040):
        base.visual(
            Cylinder(radius=0.008, length=0.030),
            origin=Origin(xyz=(x_pos, -0.050, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"hinge_lug_{0 if x_pos < 0 else 1}",
        )
        base.visual(
            Box((0.024, 0.020, 0.016)),
            origin=Origin(xyz=(x_pos, -0.040, 0.047)),
            material=body_metal,
            name=f"hinge_support_{0 if x_pos < 0 else 1}",
        )
    for x_pos in (-0.059, 0.059):
        base.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(xyz=(x_pos, -0.050, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"hinge_pin_end_{0 if x_pos < 0 else 1}",
        )
    for x_pos in (-0.090, 0.090):
        base.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(x_pos, -0.035, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bumper,
            name=f"foot_{0 if x_pos < 0 else 1}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.290, 0.150, 0.070)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    handle = model.part("handle")

    handle_mesh = mesh_from_geometry(
        section_loft(
            [
                xz_section(0.070, 0.028, 0.010, 0.004, 0.024),
                xz_section(0.095, 0.034, 0.012, 0.080, 0.025),
                xz_section(0.072, 0.020, 0.008, 0.160, 0.022),
            ]
        ),
        "handle_shell",
    )
    handle.visual(handle_mesh, material=handle_finish, name="handle_shell")
    handle.visual(
        Cylinder(radius=0.007, length=0.042),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.028, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, 0.012, 0.008)),
        material=handle_finish,
        name="rear_bridge",
    )
    handle.visual(
        Box((0.040, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.150, -0.007)),
        material=handle_finish,
        name="press_nose",
    )
    handle.visual(
        Box((0.026, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, 0.135, 0.005)),
        material=handle_finish,
        name="nose_bridge",
    )
    handle.visual(
        Box((0.052, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.118, 0.028)),
        material=bumper,
        name="grip_pad",
    )
    handle.visual(
        Box((0.032, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.118, 0.015)),
        material=handle_finish,
        name="grip_mount",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.100, 0.170, 0.040)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.080, 0.014)),
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0.0, -0.050, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    def add_stop(name: str, x_pos: float):
        stop = model.part(name)
        stop.visual(
            Box((0.020, 0.016, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=stop_plastic,
            name="shoe",
        )
        stop.visual(
            Box((0.010, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, 0.004, 0.013)),
            material=stop_plastic,
            name="web",
        )
        stop.visual(
            Box((0.008, 0.006, 0.030)),
            origin=Origin(xyz=(0.0, 0.016, 0.015)),
            material=stop_plastic,
            name="stop_face",
        )
        stop.visual(
            Cylinder(radius=0.0055, length=0.010),
            origin=Origin(xyz=(0.0, 0.012, 0.035)),
            material=steel,
            name="knob",
        )
        stop.inertial = Inertial.from_geometry(
            Box((0.022, 0.022, 0.040)),
            mass=0.06,
            origin=Origin(xyz=(0.0, 0.008, 0.020)),
        )
        model.articulation(
            f"base_to_{name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=stop,
            origin=Origin(xyz=(x_pos, 0.042, 0.040)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.15,
                lower=-0.040,
                upper=0.040,
            ),
        )

    add_stop("stop_0", -0.070)
    add_stop("stop_1", 0.070)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    hinge = object_model.get_articulation("base_to_handle")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="press_nose",
        negative_elem="die_bridge",
        min_gap=0.0,
        max_gap=0.008,
        name="handle nose rests just above die bridge",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="x",
        elem_a="press_nose",
        elem_b="die_bridge",
        min_overlap=0.025,
        name="handle nose stays centered over die bridge",
    )

    closed_box = ctx.part_element_world_aabb(handle, elem="press_nose")
    with ctx.pose({hinge: math.radians(55.0)}):
        open_box = ctx.part_element_world_aabb(handle, elem="press_nose")
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="press_nose",
            negative_elem="die_bridge",
            min_gap=0.035,
            name="opened handle lifts away from die bridge",
        )
    ctx.check(
        "handle opens upward",
        closed_box is not None
        and open_box is not None
        and open_box[0][2] > closed_box[0][2] + 0.05,
        details=f"closed={closed_box}, open={open_box}",
    )

    stop_0 = object_model.get_part("stop_0")
    stop_1 = object_model.get_part("stop_1")
    stop_0_slide = object_model.get_articulation("base_to_stop_0")
    stop_1_slide = object_model.get_articulation("base_to_stop_1")

    for stop_name, stop in (("stop_0", stop_0), ("stop_1", stop_1)):
        ctx.expect_gap(
            stop,
            base,
            axis="z",
            positive_elem="shoe",
            negative_elem="guide_rail",
            max_gap=0.001,
            max_penetration=1e-6,
            name=f"{stop_name} shoe rides on guide rail",
        )
        ctx.expect_overlap(
            stop,
            base,
            axes="x",
            elem_a="shoe",
            elem_b="guide_rail",
            min_overlap=0.018,
            name=f"{stop_name} remains captured along the rail span",
        )

    rest_stop_0 = ctx.part_world_position(stop_0)
    rest_stop_1 = ctx.part_world_position(stop_1)
    with ctx.pose({stop_0_slide: 0.030}):
        moved_stop_0 = ctx.part_world_position(stop_0)
        still_stop_1 = ctx.part_world_position(stop_1)
    with ctx.pose({stop_1_slide: -0.030}):
        moved_stop_1 = ctx.part_world_position(stop_1)
        still_stop_0 = ctx.part_world_position(stop_0)

    ctx.check(
        "stop_0 slides independently",
        rest_stop_0 is not None
        and moved_stop_0 is not None
        and still_stop_1 is not None
        and moved_stop_0[0] > rest_stop_0[0] + 0.025
        and abs(still_stop_1[0] - rest_stop_1[0]) < 1e-6,
        details=f"rest_0={rest_stop_0}, moved_0={moved_stop_0}, rest_1={rest_stop_1}, still_1={still_stop_1}",
    )
    ctx.check(
        "stop_1 slides independently",
        rest_stop_1 is not None
        and moved_stop_1 is not None
        and still_stop_0 is not None
        and moved_stop_1[0] < rest_stop_1[0] - 0.025
        and abs(still_stop_0[0] - rest_stop_0[0]) < 1e-6,
        details=f"rest_1={rest_stop_1}, moved_1={moved_stop_1}, rest_0={rest_stop_0}, still_0={still_stop_0}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
