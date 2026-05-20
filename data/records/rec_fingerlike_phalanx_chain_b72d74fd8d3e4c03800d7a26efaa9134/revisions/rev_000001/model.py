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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(radius: float, width: float, center: tuple[float, float, float]) -> cq.Workplane:
    """Cylinder with its axis along local/world Y."""
    return cq.Workplane("XZ").center(center[0], center[2]).circle(radius).extrude(width * 0.5, both=True).translate((0.0, center[1], 0.0))


def _rounded_block(size: tuple[float, float, float], center: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = _box(size, center)
    if radius > 0.0:
        try:
            shape = shape.edges("|Z").fillet(radius)
        except Exception:
            pass
    return shape


def _curved_web(
    length: float,
    drop: float,
    start_x: float,
    end_x: float,
    width_y: float,
    start_thickness: float,
    end_thickness: float,
) -> cq.Workplane:
    top: list[tuple[float, float]] = []
    bottom: list[tuple[float, float]] = []
    samples = 12
    for i in range(samples + 1):
        t = i / samples
        x = start_x + (end_x - start_x) * t
        z_center = -drop * (1.0 - math.cos(math.pi * t)) * 0.5
        half = 0.5 * (start_thickness + (end_thickness - start_thickness) * t)
        top.append((x, z_center + half))
        bottom.append((x, z_center - half))

    profile = top + list(reversed(bottom))
    return cq.Workplane("XZ").polyline(profile).close().extrude(width_y * 0.5, both=True)


def _root_housing_shape() -> cq.Workplane:
    base = _rounded_block((0.110, 0.135, 0.070), (-0.095, 0.0, -0.010), 0.006)
    rear_flange = _rounded_block((0.045, 0.155, 0.092), (-0.145, 0.0, -0.006), 0.005)
    top_rail = _rounded_block((0.100, 0.018, 0.012), (-0.090, 0.057, 0.032), 0.002)
    bottom_rail = _rounded_block((0.100, 0.018, 0.012), (-0.090, -0.057, 0.032), 0.002)

    cheek_a = _rounded_block((0.058, 0.021, 0.066), (-0.006, 0.050, 0.000), 0.004)
    cheek_b = _rounded_block((0.058, 0.021, 0.066), (-0.006, -0.050, 0.000), 0.004)
    cheek_lip_a = _y_cylinder(0.022, 0.021, (0.000, 0.050, 0.000))
    cheek_lip_b = _y_cylinder(0.022, 0.021, (0.000, -0.050, 0.000))
    bridge = _rounded_block((0.016, 0.118, 0.020), (-0.035, 0.0, -0.027), 0.002)

    return (
        base.union(rear_flange)
        .union(top_rail)
        .union(bottom_rail)
        .union(cheek_a)
        .union(cheek_b)
        .union(cheek_lip_a)
        .union(cheek_lip_b)
        .union(bridge)
    )


def _root_link_shape() -> cq.Workplane:
    body = _rounded_block((0.070, 0.056, 0.039), (0.052, 0.0, 0.000), 0.005)
    proximal_boss = _y_cylinder(0.025, 0.060, (0.000, 0.0, 0.000))
    proximal_neck = _rounded_block((0.028, 0.054, 0.034), (0.020, 0.0, 0.000), 0.003)

    fork_bridge = _rounded_block((0.016, 0.098, 0.032), (0.074, 0.0, 0.000), 0.003)
    cheek_a = _rounded_block((0.044, 0.019, 0.040), (0.101, 0.041, 0.000), 0.003)
    cheek_b = _rounded_block((0.044, 0.019, 0.040), (0.101, -0.041, 0.000), 0.003)
    eye_a = _y_cylinder(0.022, 0.019, (0.110, 0.041, 0.000))
    eye_b = _y_cylinder(0.022, 0.019, (0.110, -0.041, 0.000))

    lightening_pocket = _rounded_block((0.034, 0.060, 0.014), (0.052, 0.0, 0.014), 0.002)

    return (
        body.union(proximal_boss)
        .union(proximal_neck)
        .union(fork_bridge)
        .union(cheek_a)
        .union(cheek_b)
        .union(eye_a)
        .union(eye_b)
        .cut(lightening_pocket)
    )


def _mid_link_shape() -> cq.Workplane:
    web = _curved_web(
        length=0.090,
        drop=0.020,
        start_x=0.015,
        end_x=0.074,
        width_y=0.042,
        start_thickness=0.028,
        end_thickness=0.023,
    )
    proximal_boss = _y_cylinder(0.021, 0.050, (0.000, 0.0, 0.000))
    proximal_neck = _rounded_block((0.026, 0.040, 0.026), (0.020, 0.0, -0.001), 0.003)

    distal_bridge = _rounded_block((0.014, 0.076, 0.026), (0.069, 0.0, -0.017), 0.002)
    cheek_a = _rounded_block((0.037, 0.016, 0.032), (0.087, 0.032, -0.020), 0.0025)
    cheek_b = _rounded_block((0.037, 0.016, 0.032), (0.087, -0.032, -0.020), 0.0025)
    eye_a = _y_cylinder(0.0185, 0.016, (0.090, 0.032, -0.020))
    eye_b = _y_cylinder(0.0185, 0.016, (0.090, -0.032, -0.020))

    return web.union(proximal_boss).union(proximal_neck).union(distal_bridge).union(cheek_a).union(cheek_b).union(eye_a).union(eye_b)


def _distal_link_shape() -> cq.Workplane:
    boss = _y_cylinder(0.0175, 0.038, (0.000, 0.0, 0.000))
    neck = _rounded_block((0.020, 0.036, 0.024), (0.016, 0.0, 0.000), 0.0025)
    shank_profile = [(0.012, 0.014), (0.060, 0.016), (0.068, -0.012), (0.012, -0.012)]
    shank = cq.Workplane("XZ").polyline(shank_profile).close().extrude(0.036, both=True)
    carrier = _rounded_block((0.034, 0.050, 0.040), (0.083, 0.0, -0.001), 0.003)
    return boss.union(neck).union(shank).union(carrier)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_grasping_finger")

    aluminum = model.material("clear_anodized_aluminum", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_aluminum = model.material("dark_hard_anodized_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.76, 0.70, 1.0))
    rubber = model.material("replaceable_black_rubber", rgba=(0.015, 0.015, 0.013, 1.0))
    amber = model.material("bronze_bushing", rgba=(0.83, 0.56, 0.22, 1.0))

    housing = model.part("machine_base")
    housing.visual(
        mesh_from_cadquery(
            _rounded_block((0.110, 0.135, 0.070), (-0.095, 0.0, -0.010), 0.006)
            .union(_rounded_block((0.045, 0.155, 0.092), (-0.145, 0.0, -0.006), 0.005))
            .union(_rounded_block((0.100, 0.018, 0.012), (-0.090, 0.057, 0.032), 0.002))
            .union(_rounded_block((0.100, 0.018, 0.012), (-0.090, -0.057, 0.032), 0.002))
            .union(_rounded_block((0.016, 0.118, 0.020), (-0.035, 0.0, -0.027), 0.002)),
            "housing_body",
            tolerance=0.0008,
        ),
        material=dark_aluminum,
        name="housing_body",
    )
    for side, y in (("upper", 0.050), ("lower", -0.050)):
        housing.visual(
            mesh_from_cadquery(
                _rounded_block((0.058, 0.021, 0.066), (-0.006, y, 0.000), 0.004).union(
                    _y_cylinder(0.022, 0.021, (0.000, y, 0.000))
                ),
                f"base_clevis_{side}",
                tolerance=0.0008,
            ),
            material=dark_aluminum,
            name=f"base_clevis_{side}",
        )
        housing.visual(
            Cylinder(radius=0.0105, length=0.003),
            origin=Origin(xyz=(0.0, y + (0.012 if y > 0 else -0.012), 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"base_pin_cap_{side}",
        )
        housing.visual(
            Cylinder(radius=0.015, length=0.0025),
            origin=Origin(xyz=(0.0, y - (0.011 if y > 0 else -0.011), 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=amber,
            name=f"base_bushing_face_{side}",
        )
    for x in (-0.137, -0.063):
        for y in (-0.043, 0.043):
            housing.visual(
                Cylinder(radius=0.0065, length=0.003),
                origin=Origin(xyz=(x, y, 0.0385)),
                material=steel,
                name=f"cap_screw_{x:+.3f}_{y:+.3f}",
            )
    root_link = model.part("root_link")
    root_link.visual(
        mesh_from_cadquery(_y_cylinder(0.025, 0.084, (0.000, 0.0, 0.000)), "root_proximal_boss", tolerance=0.0008),
        material=aluminum,
        name="root_proximal_boss",
    )
    root_link.visual(
        mesh_from_cadquery(
            _rounded_block((0.070, 0.056, 0.039), (0.052, 0.0, 0.000), 0.005)
            .union(_rounded_block((0.028, 0.054, 0.034), (0.020, 0.0, 0.000), 0.003))
            .cut(_rounded_block((0.034, 0.060, 0.014), (0.052, 0.0, 0.014), 0.002)),
            "root_body",
            tolerance=0.0008,
        ),
        material=aluminum,
        name="root_body",
    )
    root_link.visual(
        mesh_from_cadquery(_rounded_block((0.016, 0.098, 0.032), (0.074, 0.0, 0.000), 0.003), "root_fork_bridge", tolerance=0.0008),
        material=aluminum,
        name="root_fork_bridge",
    )
    for side, y in (("upper", 0.041), ("lower", -0.041)):
        root_link.visual(
            mesh_from_cadquery(
                _rounded_block((0.044, 0.019, 0.040), (0.101, y, 0.000), 0.003).union(
                    _y_cylinder(0.022, 0.019, (0.110, y, 0.000))
                ),
                f"root_fork_{side}",
                tolerance=0.0008,
            ),
            material=aluminum,
            name=f"root_fork_{side}",
        )
        root_link.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(0.110, y + (0.011 if y > 0 else -0.011), 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"mid_pin_cap_{side}",
        )
    for side, y in (("upper", 0.042), ("lower", -0.042)):
        root_link.visual(
            Cylinder(radius=0.0135, length=0.003),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=amber,
            name=f"root_bushing_face_{side}",
        )

    mid_link = model.part("mid_link")
    mid_link.visual(
        mesh_from_cadquery(_y_cylinder(0.021, 0.066, (0.000, 0.0, 0.000)), "mid_proximal_boss", tolerance=0.0008),
        material=aluminum,
        name="mid_proximal_boss",
    )
    mid_link.visual(
        mesh_from_cadquery(
            _curved_web(0.090, 0.020, 0.015, 0.074, 0.042, 0.028, 0.023).union(
                _rounded_block((0.026, 0.040, 0.026), (0.020, 0.0, -0.001), 0.003)
            ),
            "mid_web",
            tolerance=0.0008,
        ),
        material=aluminum,
        name="mid_web",
    )
    mid_link.visual(
        mesh_from_cadquery(_rounded_block((0.014, 0.076, 0.026), (0.069, 0.0, -0.017), 0.002), "mid_fork_bridge", tolerance=0.0008),
        material=aluminum,
        name="mid_fork_bridge",
    )
    for side, y in (("upper", 0.032), ("lower", -0.032)):
        mid_link.visual(
            mesh_from_cadquery(
                _rounded_block((0.037, 0.016, 0.032), (0.087, y, -0.020), 0.0025).union(
                    _y_cylinder(0.0185, 0.016, (0.090, y, -0.020))
                ),
                f"mid_fork_{side}",
                tolerance=0.0008,
            ),
            material=aluminum,
            name=f"mid_fork_{side}",
        )
        mid_link.visual(
            Cylinder(radius=0.0085, length=0.0028),
            origin=Origin(xyz=(0.090, y + (0.009 if y > 0 else -0.009), -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"distal_pin_cap_{side}",
        )
    for side, y in (("upper", 0.033), ("lower", -0.033)):
        mid_link.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=amber,
            name=f"mid_bushing_face_{side}",
        )

    distal = model.part("distal_link")
    distal.visual(
        mesh_from_cadquery(_y_cylinder(0.0175, 0.050, (0.000, 0.0, 0.000)), "distal_boss", tolerance=0.0008),
        material=aluminum,
        name="distal_boss",
    )
    distal.visual(
        mesh_from_cadquery(
            _rounded_block((0.020, 0.036, 0.024), (0.016, 0.0, 0.000), 0.0025)
            .union(
                cq.Workplane("XZ")
                .polyline([(0.012, 0.014), (0.060, 0.016), (0.068, -0.012), (0.012, -0.012)])
                .close()
                .extrude(0.018, both=True)
            )
            .union(_rounded_block((0.034, 0.050, 0.040), (0.083, 0.0, -0.001), 0.003)),
            "distal_shank",
            tolerance=0.0008,
        ),
        material=aluminum,
        name="distal_shank",
    )
    for side, y in (("upper", 0.025), ("lower", -0.025)):
        distal.visual(
            Cylinder(radius=0.009, length=0.0025),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=amber,
            name=f"distal_bushing_face_{side}",
        )
    distal.visual(
        Box((0.007, 0.054, 0.044)),
        origin=Origin(xyz=(0.103, 0.0, -0.001)),
        material=rubber,
        name="tip_pad",
    )
    for y in (-0.016, 0.016):
        for z in (-0.014, 0.012):
            distal.visual(
                Cylinder(radius=0.0022, length=0.0025),
                origin=Origin(xyz=(0.107, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"pad_screw_{y:+.3f}_{z:+.3f}",
            )

    model.articulation(
        "base_to_root",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=root_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.90, effort=45.0, velocity=2.2),
    )
    model.articulation(
        "root_to_mid",
        ArticulationType.REVOLUTE,
        parent=root_link,
        child=mid_link,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=30.0, velocity=2.6),
    )
    model.articulation(
        "mid_to_distal",
        ArticulationType.REVOLUTE,
        parent=mid_link,
        child=distal,
        origin=Origin(xyz=(0.090, 0.0, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=18.0, velocity=3.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("machine_base")
    root_link = object_model.get_part("root_link")
    mid_link = object_model.get_part("mid_link")
    distal = object_model.get_part("distal_link")
    base_to_root = object_model.get_articulation("base_to_root")
    root_to_mid = object_model.get_articulation("root_to_mid")
    mid_to_distal = object_model.get_articulation("mid_to_distal")

    joints = (base_to_root, root_to_mid, mid_to_distal)
    ctx.check(
        "three serial planar revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(round(v, 6) for v in (j.axis or ())) == (0.0, 1.0, 0.0) for j in joints),
        details="The finger should use exactly three Y-axis revolute joints in one bending plane.",
    )

    ctx.expect_gap(
        root_link,
        housing,
        axis="x",
        positive_elem="root_body",
        negative_elem="housing_body",
        max_penetration=0.0,
        name="root link clears fixed machine housing",
    )
    ctx.expect_gap(
        mid_link,
        root_link,
        axis="x",
        positive_elem="mid_web",
        negative_elem="root_body",
        max_penetration=0.0,
        name="curved link clears blocky root fork",
    )
    ctx.expect_gap(
        distal,
        mid_link,
        axis="x",
        positive_elem="distal_shank",
        negative_elem="mid_web",
        max_penetration=0.0,
        name="distal segment clears mid-link fork",
    )

    root_aabb = ctx.part_world_aabb(root_link)
    mid_aabb = ctx.part_world_aabb(mid_link)
    distal_aabb = ctx.part_world_aabb(distal)
    if root_aabb and mid_aabb and distal_aabb:
        root_y = root_aabb[1][1] - root_aabb[0][1]
        mid_y = mid_aabb[1][1] - mid_aabb[0][1]
        distal_y = distal_aabb[1][1] - distal_aabb[0][1]
        ctx.check(
            "links step down in scale",
            root_y > mid_y > distal_y,
            details=f"root_y={root_y:.4f}, mid_y={mid_y:.4f}, distal_y={distal_y:.4f}",
        )
    else:
        ctx.fail("links step down in scale", "Could not compute link AABBs.")

    rest_tip = ctx.part_world_position(distal)
    with ctx.pose({base_to_root: 0.72, root_to_mid: 0.78, mid_to_distal: 0.85}):
        curled_tip = ctx.part_world_position(distal)
        ctx.expect_gap(
            distal,
            housing,
            axis="x",
            positive_elem="distal_shank",
            negative_elem="housing_body",
            min_gap=0.010,
            name="curled distal link remains ahead of machine base",
        )
        ctx.expect_gap(
            mid_link,
            housing,
            axis="x",
            positive_elem="mid_web",
            negative_elem="housing_body",
            min_gap=0.006,
            name="curled mid link remains clear of fixed housing",
        )

    ctx.check(
        "curl pose moves fingertip inward",
        rest_tip is not None
        and curled_tip is not None
        and curled_tip[2] < rest_tip[2] - 0.060
        and curled_tip[0] < rest_tip[0] + 0.010,
        details=f"rest_tip={rest_tip}, curled_tip={curled_tip}",
    )

    return ctx.report()


object_model = build_object_model()
