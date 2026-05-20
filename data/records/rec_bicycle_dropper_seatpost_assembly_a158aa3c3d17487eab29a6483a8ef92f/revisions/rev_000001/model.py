from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
):
    """Simple hollow tube, authored from z=0 to z=length."""
    shape = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return mesh_from_cadquery(shape, name, tolerance=0.00045, angular_tolerance=0.08)


def _ring_shape(outer_radius: float, inner_radius: float, length: float, z_bottom: float):
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, z_bottom))
    )


def _boot_mesh(name: str):
    """Corrugated rubber bellows that clears the sliding stanchion."""
    inner_radius = 0.0134
    boot = _ring_shape(0.0156, inner_radius, 0.064, 0.0)
    # Raised ribs give the soft boot a believable accordion profile while the
    # thin inner sleeve keeps all ribs connected as one physical rubber part.
    for z in (0.004, 0.016, 0.028, 0.040, 0.052):
        boot = boot.union(_ring_shape(0.0183, inner_radius, 0.007, z))
    boot = boot.union(_ring_shape(0.0192, inner_radius, 0.009, -0.001))
    boot = boot.union(_ring_shape(0.0172, inner_radius, 0.008, 0.057))
    return mesh_from_cadquery(boot, name, tolerance=0.0005, angular_tolerance=0.08)


def _cylinder_between(part, start, end, radius, *, material: Material, name: str):
    """Add a cylinder whose local +Z axis spans start -> end in part coordinates."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")
    ux, uy, uz = vx / length, vy / length, vz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="external_cable_dropper_seatpost")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.03, 0.033, 0.035, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    stanchion = model.material("brushed_stanchion", rgba=(0.62, 0.64, 0.62, 1.0))
    bronze = model.material("bronze_bushing", rgba=(0.72, 0.48, 0.18, 1.0))
    steel = model.material("stainless_bolts", rgba=(0.72, 0.72, 0.69, 1.0))
    label = model.material("muted_gray_label", rgba=(0.25, 0.27, 0.27, 1.0))

    outer = model.part("outer_sleeve")
    outer.visual(
        _tube_mesh(0.0155, 0.0136, 0.300, "outer_post_sleeve"),
        material=matte_black,
        name="outer_tube",
    )
    outer.visual(
        _tube_mesh(0.0188, 0.0129, 0.027, "top_wiper_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        material=satin_black,
        name="top_collar",
    )
    outer.visual(
        _tube_mesh(0.0147, 0.0118, 0.040, "linear_bushing"),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=bronze,
        name="linear_bushing",
    )
    outer.visual(
        _boot_mesh("rubber_gap_boot"),
        origin=Origin(xyz=(0.0, 0.0, 0.314)),
        material=rubber,
        name="rubber_boot",
    )
    outer.visual(
        Box((0.0020, 0.020, 0.085)),
        origin=Origin(xyz=(0.0159, 0.0, 0.160)),
        material=label,
        name="sleeve_logo_panel",
    )
    # Cable stop and lower guide clamp are integral to the fixed outer sleeve.
    outer.visual(
        Box((0.043, 0.013, 0.011)),
        origin=Origin(xyz=(0.037, 0.0, 0.176)),
        material=satin_black,
        name="cable_stop",
    )
    outer.visual(
        Box((0.043, 0.012, 0.009)),
        origin=Origin(xyz=(0.037, 0.0, 0.025)),
        material=satin_black,
        name="cable_guide",
    )
    _cylinder_between(
        outer,
        (0.056, 0.0, 0.028),
        (0.056, 0.0, 0.190),
        0.0033,
        material=rubber,
        name="cable_housing",
    )
    _cylinder_between(
        outer,
        (0.056, 0.0, 0.176),
        (0.032, 0.0, 0.118),
        0.0013,
        material=steel,
        name="exposed_cable",
    )
    # Two yoke tabs around the lever hub, plus a bridge that visibly grows out
    # of the lower sleeve wall.
    outer.visual(
        Box((0.014, 0.005, 0.036)),
        origin=Origin(xyz=(0.027, 0.0115, 0.105)),
        material=satin_black,
        name="pivot_tab_0",
    )
    outer.visual(
        Box((0.014, 0.005, 0.036)),
        origin=Origin(xyz=(0.027, -0.0115, 0.105)),
        material=satin_black,
        name="pivot_tab_1",
    )
    outer.visual(
        Box((0.013, 0.026, 0.006)),
        origin=Origin(xyz=(0.0205, 0.0, 0.119)),
        material=satin_black,
        name="pivot_bridge",
    )

    inner = model.part("inner_post")
    inner.visual(
        Cylinder(radius=0.0120, length=0.450),
        # The child frame is at the sleeve entry.  The post extends below it so
        # it remains retained inside the bushing at full travel.
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=stanchion,
        name="inner_tube",
    )
    inner.visual(
        Cylinder(radius=0.0150, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.323)),
        material=satin_black,
        name="clamp_base_collar",
    )
    inner.visual(
        Box((0.036, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=satin_black,
        name="clamp_head",
    )
    inner.visual(
        Box((0.076, 0.046, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=satin_black,
        name="lower_cradle",
    )
    # Saddle rails captured between the lower and upper cradles.
    for idx, y in enumerate((-0.014, 0.014)):
        inner.visual(
            Cylinder(radius=0.0035, length=0.090),
            origin=Origin(xyz=(0.0, y, 0.355), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"saddle_rail_{idx}",
        )
    inner.visual(
        Box((0.074, 0.044, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.3625)),
        material=satin_black,
        name="upper_cradle",
    )
    for idx, x in enumerate((-0.025, 0.025)):
        inner.visual(
            Cylinder(radius=0.0030, length=0.038),
            origin=Origin(xyz=(x, 0.0, 0.354)),
            material=steel,
            name=f"clamp_bolt_{idx}",
        )
        inner.visual(
            Cylinder(radius=0.0054, length=0.0045),
            origin=Origin(xyz=(x, 0.0, 0.371)),
            material=steel,
            name=f"bolt_head_{idx}",
        )
        inner.visual(
            Cylinder(radius=0.0048, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.336)),
            material=steel,
            name=f"barrel_nut_{idx}",
        )

    lever = model.part("cable_lever")
    lever.visual(
        Cylinder(radius=0.0060, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lever_hub",
    )
    _cylinder_between(
        lever,
        (0.004, 0.0, -0.004),
        (0.017, 0.0, -0.056),
        0.0030,
        material=satin_black,
        name="lever_blade",
    )
    lever.visual(
        Sphere(radius=0.0048),
        origin=Origin(xyz=(0.018, 0.0, -0.058)),
        material=steel,
        name="cable_anchor",
    )
    _cylinder_between(
        lever,
        (0.017, 0.0, -0.055),
        (0.025, 0.0, -0.083),
        0.0012,
        material=steel,
        name="cable_tail",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.100),
    )
    model.articulation(
        "sleeve_to_lever",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=lever,
        origin=Origin(xyz=(0.027, 0.0, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=0.0, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_sleeve")
    inner = object_model.get_part("inner_post")
    lever = object_model.get_part("cable_lever")
    slide = object_model.get_articulation("sleeve_to_post")
    lever_joint = object_model.get_articulation("sleeve_to_lever")

    ctx.allow_overlap(
        outer,
        inner,
        elem_a="linear_bushing",
        elem_b="inner_tube",
        reason=(
            "The bronze linear bushing is modeled with a tiny local interference "
            "fit against the sliding post to show the close bearing support."
        ),
    )

    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="linear_bushing",
        margin=0.001,
        name="inner post is close-fit in the linear bushing",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="inner_tube",
        elem_b="linear_bushing",
        min_overlap=0.035,
        name="inner post rides through the bushing at rest",
    )
    ctx.expect_overlap(
        outer,
        inner,
        axes="z",
        elem_a="rubber_boot",
        elem_b="inner_tube",
        min_overlap=0.050,
        name="rubber boot covers the annular stanchion gap",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({slide: 0.100}):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="linear_bushing",
            margin=0.001,
            name="extended post remains centered in the bushing",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="inner_tube",
            elem_b="linear_bushing",
            min_overlap=0.030,
            name="extended post retains insertion in the bushing",
        )
        extended_pos = ctx.part_world_position(inner)
    ctx.check(
        "dropper post slides upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.095,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_anchor = ctx.part_element_world_aabb(lever, elem="cable_anchor")
    with ctx.pose({lever_joint: 0.65}):
        pulled_anchor = ctx.part_element_world_aabb(lever, elem="cable_anchor")
    ctx.check(
        "cable lever swings outward from the sleeve",
        rest_anchor is not None
        and pulled_anchor is not None
        and pulled_anchor[1][0] > rest_anchor[1][0] + 0.025,
        details=f"rest={rest_anchor}, pulled={pulled_anchor}",
    )

    return ctx.report()


object_model = build_object_model()
