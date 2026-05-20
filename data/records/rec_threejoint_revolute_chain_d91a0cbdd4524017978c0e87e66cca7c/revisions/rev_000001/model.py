from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, center[1], 0.0))
    )


def _make_clevis_link(length: float) -> cq.Workplane:
    """A deep industrial link: proximal tongue, box-section web, distal fork."""
    body_h = 0.115
    tongue_w = 0.074
    lug_w = 0.034
    lug_y = 0.065
    boss_r = 0.078
    bore_r = 0.027
    fork_len = 0.155

    solid = _cyl_y(boss_r, tongue_w, (0.0, 0.0, 0.0))
    solid = solid.union(_box((length - fork_len + 0.010, tongue_w, body_h), ((length - fork_len) / 2.0, 0.0, 0.0)))

    # Raised top and bottom flanges make the web read as a deeper fabricated section.
    flange_len = length - fork_len - 0.070
    solid = solid.union(_box((flange_len, tongue_w + 0.010, 0.018), (flange_len / 2.0 + 0.050, 0.0, body_h / 2.0 + 0.005)))
    solid = solid.union(_box((flange_len, tongue_w + 0.010, 0.018), (flange_len / 2.0 + 0.050, 0.0, -body_h / 2.0 - 0.005)))

    # Cross bridge sets up the clevis cheeks without filling the child tongue space.
    bridge_x = length - fork_len + 0.018
    solid = solid.union(_box((0.040, 2.0 * lug_y + lug_w, body_h), (bridge_x, 0.0, 0.0)))

    for y in (-lug_y, lug_y):
        solid = solid.union(_box((fork_len, lug_w, body_h), (length - fork_len / 2.0, y, 0.0)))
        solid = solid.union(_cyl_y(boss_r, lug_w, (length, y, 0.0)))
        # Local circular reinforcement pad on each outer cheek face.
        solid = solid.union(_cyl_y(boss_r * 0.78, 0.012, (length, y + math.copysign(lug_w / 2.0 + 0.006, y), 0.0)))

    for x in (0.0, length):
        solid = solid.cut(_cyl_y(bore_r, 0.260, (x, 0.0, 0.0)))

    return solid


def _make_link_body(length: float) -> cq.Workplane:
    body_h = 0.115
    tongue_w = 0.074
    boss_r = 0.078
    bore_r = 0.027
    fork_len = 0.155

    body_end = length - fork_len + 0.040
    solid = _cyl_y(boss_r, tongue_w, (0.0, 0.0, 0.0))
    solid = solid.union(_box((body_end + 0.006, tongue_w, body_h), (body_end / 2.0, 0.0, 0.0)))

    flange_len = body_end - 0.070
    solid = solid.union(_box((flange_len, tongue_w + 0.010, 0.018), (flange_len / 2.0 + 0.050, 0.0, body_h / 2.0 + 0.005)))
    solid = solid.union(_box((flange_len, tongue_w + 0.010, 0.018), (flange_len / 2.0 + 0.050, 0.0, -body_h / 2.0 - 0.005)))
    solid = solid.cut(_cyl_y(bore_r, 0.150, (0.0, 0.0, 0.0)))
    return solid


def _make_link_bridge(length: float) -> cq.Workplane:
    body_h = 0.115
    lug_w = 0.034
    lug_y = 0.065
    fork_len = 0.155
    bridge_x = length - fork_len + 0.020
    return _box((0.048, 2.0 * lug_y + lug_w, body_h), (bridge_x, 0.0, 0.0))


def _make_link_lug(length: float, y: float) -> cq.Workplane:
    body_h = 0.115
    lug_w = 0.034
    boss_r = 0.078
    bore_r = 0.027
    fork_len = 0.155

    solid = _box((fork_len, lug_w, body_h), (length - fork_len / 2.0, y, 0.0))
    solid = solid.union(_cyl_y(boss_r, lug_w, (length, y, 0.0)))
    solid = solid.union(_cyl_y(boss_r * 0.78, 0.012, (length, y + math.copysign(lug_w / 2.0 + 0.006, y), 0.0)))
    solid = solid.cut(_cyl_y(bore_r, lug_w + 0.030, (length, y, 0.0)))
    return solid


def _make_end_tab(length: float) -> cq.Workplane:
    body_h = 0.105
    tongue_w = 0.074
    boss_r = 0.076
    bore_r = 0.027

    solid = _cyl_y(boss_r, tongue_w, (0.0, 0.0, 0.0))
    solid = solid.union(_box((length - 0.030, tongue_w, body_h), ((length - 0.030) / 2.0, 0.0, 0.0)))
    solid = solid.union(_cyl_y(0.052, tongue_w, (length, 0.0, 0.0)))
    solid = solid.union(_box((0.070, tongue_w + 0.010, 0.018), (length - 0.035, 0.0, body_h / 2.0 + 0.004)))
    solid = solid.union(_box((0.070, tongue_w + 0.010, 0.018), (length - 0.035, 0.0, -body_h / 2.0 - 0.004)))

    solid = solid.cut(_cyl_y(bore_r, 0.220, (0.0, 0.0, 0.0)))
    solid = solid.cut(_cyl_y(0.018, 0.220, (length, 0.0, 0.0)))
    return solid


def _make_base_cheek(joint_z: float, y: float) -> cq.Workplane:
    cheek_t = 0.036
    bore_r = 0.027
    boss_r = 0.092

    cheek = _box((0.190, cheek_t, 0.205), (0.000, y, joint_z - 0.095))
    cheek = cheek.union(_cyl_y(boss_r, cheek_t, (0.0, y, joint_z)))
    cheek = cheek.union(
        cq.Workplane("XZ")
        .polyline([(-0.120, 0.065), (0.120, 0.065), (0.060, joint_z - 0.035), (-0.050, joint_z - 0.020)])
        .close()
        .extrude(cheek_t / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )
    cheek = cheek.cut(_cyl_y(bore_r, 0.080, (0.0, y, joint_z)))
    return cheek


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_industrial_revolute_chain")

    painted_blue = model.material("machined_blue_paint", rgba=(0.12, 0.21, 0.30, 1.0))
    link_yellow = model.material("safety_yellow_paint", rgba=(0.95, 0.64, 0.13, 1.0))
    dark_rubber = model.material("dark_bore_shadow", rgba=(0.025, 0.025, 0.023, 1.0))
    pin_steel = model.material("brushed_pin_steel", rgba=(0.62, 0.62, 0.57, 1.0))

    joint_z = 0.320
    link_0_len = 0.420
    link_1_len = 0.360
    end_len = 0.225

    base = model.part("base")
    base.visual(
        Box((0.600, 0.340, 0.060)),
        origin=Origin(xyz=(0.180, 0.0, 0.030)),
        material=painted_blue,
        name="floor_box",
    )
    base.visual(
        Box((0.220, 0.225, 0.170)),
        origin=Origin(xyz=(-0.010, 0.0, 0.145)),
        material=painted_blue,
        name="pedestal_box",
    )
    base.visual(
        Box((0.175, 0.212, 0.038)),
        origin=Origin(xyz=(-0.020, 0.0, joint_z - 0.162)),
        material=painted_blue,
        name="cheek_bridge",
    )
    base.visual(
        mesh_from_cadquery(_make_base_cheek(joint_z, -0.088), "base_cheek_near", tolerance=0.0008, angular_tolerance=0.08),
        material=painted_blue,
        name="cheek_near",
    )
    base.visual(
        mesh_from_cadquery(_make_base_cheek(joint_z, 0.088), "base_cheek_far", tolerance=0.0008, angular_tolerance=0.08),
        material=painted_blue,
        name="cheek_far",
    )
    base.visual(
        Cylinder(radius=0.027, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, joint_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="pin_0",
    )
    for y, suffix in ((-0.108, "near"), (0.108, "far")):
        base.visual(
            Cylinder(radius=0.038, length=0.012),
            origin=Origin(xyz=(0.0, y, joint_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pin_steel,
            name=f"pin_0_cap_{suffix}",
        )

    link_0 = model.part("link_0")
    link_0.visual(
        mesh_from_cadquery(_make_link_body(link_0_len), "link_0_body", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="body",
    )
    link_0.visual(
        mesh_from_cadquery(_make_link_bridge(link_0_len), "link_0_bridge", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="fork_bridge",
    )
    link_0.visual(
        mesh_from_cadquery(_make_link_lug(link_0_len, -0.065), "link_0_lug_near", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="lug_near",
    )
    link_0.visual(
        mesh_from_cadquery(_make_link_lug(link_0_len, 0.065), "link_0_lug_far", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="lug_far",
    )
    link_0.visual(
        Cylinder(radius=0.027, length=0.205),
        origin=Origin(xyz=(link_0_len, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="pin_1",
    )
    for y, suffix in ((-0.092, "near"), (0.092, "far")):
        link_0.visual(
            Cylinder(radius=0.035, length=0.010),
            origin=Origin(xyz=(link_0_len, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pin_steel,
            name=f"pin_1_cap_{suffix}",
        )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_make_link_body(link_1_len), "link_1_body", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="body",
    )
    link_1.visual(
        mesh_from_cadquery(_make_link_bridge(link_1_len), "link_1_bridge", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="fork_bridge",
    )
    link_1.visual(
        mesh_from_cadquery(_make_link_lug(link_1_len, -0.065), "link_1_lug_near", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="lug_near",
    )
    link_1.visual(
        mesh_from_cadquery(_make_link_lug(link_1_len, 0.065), "link_1_lug_far", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="lug_far",
    )
    link_1.visual(
        Cylinder(radius=0.027, length=0.205),
        origin=Origin(xyz=(link_1_len, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="pin_2",
    )
    for y, suffix in ((-0.092, "near"), (0.092, "far")):
        link_1.visual(
            Cylinder(radius=0.035, length=0.010),
            origin=Origin(xyz=(link_1_len, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pin_steel,
            name=f"pin_2_cap_{suffix}",
        )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_make_end_tab(end_len), "end_tab_body", tolerance=0.0008, angular_tolerance=0.08),
        material=link_yellow,
        name="tab_body",
    )

    lift_axis = (0.0, -1.0, 0.0)
    model.articulation(
        "joint_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=lift_axis,
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=0.0, upper=1.70),
    )
    model.articulation(
        "joint_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link_0_len, 0.0, 0.0)),
        axis=lift_axis,
        motion_limits=MotionLimits(effort=140.0, velocity=1.4, lower=-1.35, upper=1.45),
    )
    model.articulation(
        "joint_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_tab,
        origin=Origin(xyz=(link_1_len, 0.0, 0.0)),
        axis=lift_axis,
        motion_limits=MotionLimits(effort=90.0, velocity=1.6, lower=-1.30, upper=1.40),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [object_model.get_articulation(name) for name in ("joint_0", "joint_1", "joint_2")]
    links = [object_model.get_part(name) for name in ("base", "link_0", "link_1", "end_tab")]

    ctx.check(
        "three revolute serial joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and [j.parent for j in joints] == ["base", "link_0", "link_1"]
        and [j.child for j in joints] == ["link_0", "link_1", "end_tab"],
        details=f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in object_model.articulations]}",
    )

    ctx.check(
        "hinge axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, -1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.allow_overlap(
        links[0],
        links[1],
        elem_a="pin_0",
        elem_b="body",
        reason="The steel first-joint pin is intentionally captured through the link boss bore.",
    )
    ctx.allow_overlap(
        links[1],
        links[2],
        elem_a="pin_1",
        elem_b="body",
        reason="The steel second-joint pin is intentionally captured through the next link boss bore.",
    )
    ctx.allow_overlap(
        links[2],
        links[3],
        elem_a="pin_2",
        elem_b="tab_body",
        reason="The steel third-joint pin is intentionally captured through the compact end-tab boss bore.",
    )

    # The fork-and-tongue joints are separated in Y, but they overlap in the side
    # projection around each hinge axis, which confirms the bosses are captured.
    ctx.expect_overlap(links[1], links[0], axes="xz", min_overlap=0.090, name="base cheek captures first boss")
    ctx.expect_overlap(links[2], links[1], axes="xz", min_overlap=0.090, name="first link fork captures second boss")
    ctx.expect_overlap(links[3], links[2], axes="xz", min_overlap=0.080, name="second link fork captures end tab boss")
    ctx.expect_overlap(links[0], links[1], axes="xz", elem_a="pin_0", elem_b="body", min_overlap=0.050, name="first pin passes through boss")
    ctx.expect_overlap(links[1], links[2], axes="xz", elem_a="pin_1", elem_b="body", min_overlap=0.050, name="second pin passes through boss")
    ctx.expect_overlap(links[2], links[3], axes="xz", elem_a="pin_2", elem_b="tab_body", min_overlap=0.050, name="third pin passes through boss")

    rest_aabb = ctx.part_world_aabb(links[3])
    with ctx.pose({"joint_0": 0.65, "joint_1": 0.50, "joint_2": 0.35}):
        raised_aabb = ctx.part_world_aabb(links[3])

    ctx.check(
        "positive revolute motion folds chain upward",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > rest_aabb[1][2] + 0.20,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
