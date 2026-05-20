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


def _cylinder_along_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Return a cylinder descriptor and transform whose axis is local/world Y."""

    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _bar_between(part, start, end, width_y, height_z, *, material, name):
    """Add a rectangular link web spanning two points in the XZ motion plane."""

    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    pitch = -math.atan2(dz, dx)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    part.visual(
        Box((length, width_y, height_z)),
        origin=Origin(xyz=center, rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_lever_chain")

    painted_base = Material("painted_base", color=(0.10, 0.12, 0.13, 1.0))
    dark_steel = Material("dark_blued_steel", color=(0.05, 0.09, 0.13, 1.0))
    blue_steel = Material("blue_steel", color=(0.05, 0.18, 0.36, 1.0))
    ochre_steel = Material("ochre_steel", color=(0.75, 0.49, 0.13, 1.0))
    red_handle = Material("red_handle", color=(0.72, 0.10, 0.07, 1.0))
    black_rubber = Material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    brushed_pin = Material("brushed_pin_faces", color=(0.68, 0.69, 0.66, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.28, 0.035)),
        origin=Origin(xyz=(0.055, 0.0, 0.0175)),
        material=painted_base,
        name="foot_plate",
    )
    base.visual(
        Box((0.16, 0.14, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=painted_base,
        name="raised_plinth",
    )
    base.visual(
        Box((0.12, 0.034, 0.178)),
        origin=Origin(xyz=(0.0, 0.071, 0.145)),
        material=painted_base,
        name="yoke_cheek_pos",
    )
    base.visual(
        Box((0.12, 0.034, 0.178)),
        origin=Origin(xyz=(0.0, -0.071, 0.145)),
        material=painted_base,
        name="yoke_cheek_neg",
    )
    cyl, cyl_o = _cylinder_along_y(0.074, 0.036)
    base.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.071, 0.22), rpy=cyl_o.rpy),
        material=painted_base,
        name="base_ear_pos",
    )
    base.visual(
        cyl,
        origin=Origin(xyz=(0.0, -0.071, 0.22), rpy=cyl_o.rpy),
        material=painted_base,
        name="base_ear_neg",
    )
    cyl, cyl_o = _cylinder_along_y(0.028, 0.006)
    base.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.092, 0.22), rpy=cyl_o.rpy),
        material=brushed_pin,
        name="base_pin_cap_pos",
    )
    base.visual(
        cyl,
        origin=Origin(xyz=(0.0, -0.092, 0.22), rpy=cyl_o.rpy),
        material=brushed_pin,
        name="base_pin_cap_neg",
    )
    for i, (x, y) in enumerate(((-0.14, -0.10), (0.25, -0.10), (-0.14, 0.10), (0.25, 0.10))):
        base.visual(
            Box((0.075, 0.045, 0.012)),
            origin=Origin(xyz=(x, y, 0.006)),
            material=black_rubber,
            name=f"rubber_pad_{i}",
        )

    rocker = model.part("rocker")
    cyl, cyl_o = _cylinder_along_y(0.062, 0.106)
    rocker.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_o.rpy),
        material=dark_steel,
        name="root_hub",
    )
    cyl, cyl_o = _cylinder_along_y(0.035, 0.004)
    rocker.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.051, 0.0), rpy=cyl_o.rpy),
        material=brushed_pin,
        name="root_bushing_pos",
    )
    rocker.visual(
        cyl,
        origin=Origin(xyz=(0.0, -0.051, 0.0), rpy=cyl_o.rpy),
        material=brushed_pin,
        name="root_bushing_neg",
    )
    _bar_between(rocker, (-0.004, 0.0, 0.0), (0.175, 0.0, 0.026), 0.075, 0.042, material=dark_steel, name="wide_root_web")
    rocker.visual(
        Box((0.040, 0.175, 0.042)),
        origin=Origin(xyz=(0.155, 0.0, 0.023)),
        material=dark_steel,
        name="offset_bridge",
    )
    _bar_between(rocker, (0.145, 0.066, 0.022), (0.360, 0.066, 0.055), 0.035, 0.036, material=dark_steel, name="fork_rail_pos")
    _bar_between(rocker, (0.145, -0.066, 0.022), (0.360, -0.066, 0.055), 0.035, 0.036, material=dark_steel, name="fork_rail_neg")
    cyl, cyl_o = _cylinder_along_y(0.055, 0.032)
    rocker.visual(
        cyl,
        origin=Origin(xyz=(0.360, 0.058, 0.055), rpy=cyl_o.rpy),
        material=dark_steel,
        name="tip_ear_pos",
    )
    rocker.visual(
        cyl,
        origin=Origin(xyz=(0.360, -0.058, 0.055), rpy=cyl_o.rpy),
        material=dark_steel,
        name="tip_ear_neg",
    )

    coupler = model.part("coupler")
    cyl, cyl_o = _cylinder_along_y(0.046, 0.084)
    coupler.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_o.rpy),
        material=blue_steel,
        name="root_hub",
    )
    cyl, cyl_o = _cylinder_along_y(0.026, 0.004)
    coupler.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=cyl_o.rpy),
        material=brushed_pin,
        name="root_bushing_pos",
    )
    coupler.visual(
        cyl,
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=cyl_o.rpy),
        material=brushed_pin,
        name="root_bushing_neg",
    )
    _bar_between(coupler, (-0.003, 0.0, 0.0), (0.145, 0.0, -0.014), 0.052, 0.032, material=blue_steel, name="narrow_root_web")
    coupler.visual(
        Box((0.034, 0.132, 0.032)),
        origin=Origin(xyz=(0.140, 0.0, -0.014)),
        material=blue_steel,
        name="small_bridge",
    )
    _bar_between(coupler, (0.130, 0.047, -0.014), (0.300, 0.047, -0.040), 0.026, 0.030, material=blue_steel, name="tip_rail_pos")
    _bar_between(coupler, (0.130, -0.047, -0.014), (0.300, -0.047, -0.040), 0.026, 0.030, material=blue_steel, name="tip_rail_neg")
    cyl, cyl_o = _cylinder_along_y(0.039, 0.025)
    coupler.visual(
        cyl,
        origin=Origin(xyz=(0.300, 0.041, -0.040), rpy=cyl_o.rpy),
        material=blue_steel,
        name="tip_ear_pos",
    )
    coupler.visual(
        cyl,
        origin=Origin(xyz=(0.300, -0.041, -0.040), rpy=cyl_o.rpy),
        material=blue_steel,
        name="tip_ear_neg",
    )

    tip_link = model.part("tip_link")
    cyl, cyl_o = _cylinder_along_y(0.033, 0.057)
    tip_link.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_o.rpy),
        material=ochre_steel,
        name="root_hub",
    )
    cyl, cyl_o = _cylinder_along_y(0.019, 0.004)
    tip_link.visual(
        cyl,
        origin=Origin(xyz=(0.0, 0.0265, 0.0), rpy=cyl_o.rpy),
        material=brushed_pin,
        name="root_bushing_pos",
    )
    tip_link.visual(
        cyl,
        origin=Origin(xyz=(0.0, -0.0265, 0.0), rpy=cyl_o.rpy),
        material=brushed_pin,
        name="root_bushing_neg",
    )
    _bar_between(tip_link, (0.000, 0.0, 0.0), (0.115, 0.0, 0.018), 0.040, 0.026, material=ochre_steel, name="inner_step")
    _bar_between(tip_link, (0.100, 0.0, 0.018), (0.245, 0.0, 0.074), 0.030, 0.023, material=ochre_steel, name="outer_step")
    cyl, cyl_o = _cylinder_along_y(0.032, 0.058)
    tip_link.visual(
        cyl,
        origin=Origin(xyz=(0.260, 0.0, 0.080), rpy=cyl_o.rpy),
        material=red_handle,
        name="rounded_tip",
    )

    model.articulation(
        "base_to_rocker",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.65, upper=0.85),
    )
    model.articulation(
        "rocker_to_coupler",
        ArticulationType.REVOLUTE,
        parent=rocker,
        child=coupler,
        origin=Origin(xyz=(0.360, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.10, upper=1.05),
    )
    model.articulation(
        "coupler_to_tip",
        ArticulationType.REVOLUTE,
        parent=coupler,
        child=tip_link,
        origin=Origin(xyz=(0.300, 0.0, -0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=3.0, lower=-0.95, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    rocker = object_model.get_part("rocker")
    coupler = object_model.get_part("coupler")
    tip_link = object_model.get_part("tip_link")
    j0 = object_model.get_articulation("base_to_rocker")
    j1 = object_model.get_articulation("rocker_to_coupler")
    j2 = object_model.get_articulation("coupler_to_tip")

    joints = [j0, j1, j2]
    ctx.check(
        "serial chain has three revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "all revolute axes share the motion plane normal",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    for parent, child, parent_elem, child_elem, name in (
        (base, rocker, "base_ear_pos", "root_hub", "base positive cheek contacts root hub"),
        (base, rocker, "base_ear_neg", "root_hub", "base negative cheek contacts root hub"),
        (rocker, coupler, "tip_ear_pos", "root_hub", "rocker positive fork contacts coupler"),
        (rocker, coupler, "tip_ear_neg", "root_hub", "rocker negative fork contacts coupler"),
        (coupler, tip_link, "tip_ear_pos", "root_hub", "coupler positive fork contacts tip link"),
        (coupler, tip_link, "tip_ear_neg", "root_hub", "coupler negative fork contacts tip link"),
    ):
        ctx.expect_contact(parent, child, elem_a=parent_elem, elem_b=child_elem, contact_tol=1e-5, name=name)
        ctx.expect_overlap(
            parent,
            child,
            axes="xz",
            elem_a=parent_elem,
            elem_b=child_elem,
            min_overlap=0.020,
            name=f"{name} is coaxial in the motion plane",
        )

    def y_width(part):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return aabb[1][1] - aabb[0][1]

    widths = [y_width(rocker), y_width(coupler), y_width(tip_link)]
    ctx.check(
        "link widths taper instead of repeating",
        all(w is not None for w in widths)
        and widths[0] > widths[1] + 0.025
        and widths[1] > widths[2] + 0.025,
        details=f"y_widths={widths}",
    )

    rest_tip = ctx.part_world_position(tip_link)
    with ctx.pose({j0: 0.45, j1: -0.35, j2: 0.65}):
        moved_tip = ctx.part_world_position(tip_link)
    ctx.check(
        "articulated chain moves within the XZ plane",
        rest_tip is not None
        and moved_tip is not None
        and abs(moved_tip[1] - rest_tip[1]) < 1e-6
        and math.hypot(moved_tip[0] - rest_tip[0], moved_tip[2] - rest_tip[2]) > 0.05,
        details=f"rest_tip={rest_tip}, moved_tip={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
