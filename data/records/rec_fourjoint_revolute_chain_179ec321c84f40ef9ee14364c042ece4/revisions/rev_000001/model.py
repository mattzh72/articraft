from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_bolt_pattern(
    part,
    prefix: str,
    x: float,
    y_span: float,
    z: float,
    material: Material,
    *,
    radius: float = 0.006,
    length: float = 0.004,
    x_offsets: tuple[float, float] = (-0.035, 0.035),
) -> None:
    for ix, dx in enumerate(x_offsets):
        for iy, sy in enumerate((-1.0, 1.0)):
            _cyl(
                part,
                f"{prefix}_bolt_{ix}_{iy}",
                radius,
                length,
                (x + dx, sy * y_span, z),
                material,
            )


def _add_distal_yoke(part, length: float, material_plate: Material, material_bearing: Material, material_bolt: Material) -> None:
    """Add a vertical-axis clevis/yoke at local +X for the next serial joint."""
    _box(part, "distal_upper_yoke", (0.130, 0.135, 0.016), (length, 0.0, 0.045), material_plate)
    _box(part, "distal_lower_yoke", (0.130, 0.135, 0.016), (length, 0.0, -0.045), material_plate)
    _box(part, "distal_left_spacer", (0.030, 0.012, 0.082), (length - 0.078, 0.073, 0.0), material_plate)
    _box(part, "distal_right_spacer", (0.030, 0.012, 0.082), (length - 0.078, -0.073, 0.0), material_plate)
    _box(part, "distal_web", (0.048, 0.158, 0.080), (length - 0.095, 0.0, 0.0), material_plate)
    _cyl(part, "distal_pin_shank", 0.018, 0.112, (length, 0.0, 0.0), material_bolt)
    _cyl(part, "distal_upper_boss", 0.065, 0.012, (length, 0.0, 0.059), material_bearing)
    _cyl(part, "distal_lower_boss", 0.065, 0.012, (length, 0.0, -0.059), material_bearing)
    _cyl(part, "distal_pin_head", 0.020, 0.008, (length, 0.0, 0.069), material_bolt)
    _add_bolt_pattern(part, "distal_upper", length, 0.046, 0.055, material_bolt, radius=0.0045, length=0.004)
    _add_bolt_pattern(part, "distal_lower", length, 0.046, -0.055, material_bolt, radius=0.0045, length=0.004)


def _add_box_section_link(
    part,
    *,
    length: float,
    body_material: Material,
    plate_material: Material,
    cover_material: Material,
    bearing_material: Material,
    bolt_material: Material,
    terminal: bool = False,
) -> None:
    """Rigid box-section arm segment with a proximal hub and optional distal clevis."""
    body_start = 0.070
    body_end = length - (0.110 if not terminal else 0.080)
    body_len = body_end - body_start
    body_cx = 0.5 * (body_start + body_end)

    # Proximal rotary lug captured between the parent's yoke plates.
    _cyl(part, "proximal_hub", 0.055, 0.032, (0.0, 0.0, 0.0), bearing_material)
    _cyl(part, "hub_sleeve", 0.025, 0.067, (0.0, 0.0, 0.0), bearing_material)
    _cyl(part, "upper_bearing_cap", 0.067, 0.006, (0.0, 0.0, 0.0305), bearing_material)
    _cyl(part, "lower_bearing_cap", 0.067, 0.006, (0.0, 0.0, -0.0305), bearing_material)
    _cyl(part, "center_plug", 0.022, 0.004, (0.0, 0.0, 0.033), bolt_material)
    _box(part, "proximal_neck", (0.075, 0.046, 0.044), (0.082, 0.0, 0.0), plate_material)
    _box(part, "proximal_datum", (0.018, 0.072, 0.052), (0.082, 0.0, 0.0), plate_material)

    # Rectangular-tube body: four welded/machined walls leave the center visibly hollow.
    _box(part, "upper_flange", (body_len, 0.078, 0.008), (body_cx, 0.0, 0.021), body_material)
    _box(part, "lower_flange", (body_len, 0.078, 0.008), (body_cx, 0.0, -0.021), body_material)
    _box(part, "left_web", (body_len, 0.008, 0.050), (body_cx, 0.035, 0.0), body_material)
    _box(part, "right_web", (body_len, 0.008, 0.050), (body_cx, -0.035, 0.0), body_material)
    _box(part, "mid_bulkhead", (0.014, 0.078, 0.050), (body_cx, 0.0, 0.0), plate_material)

    # Removable access covers and screw heads on two datum faces.
    cover_x = body_start + 0.42 * body_len
    _box(part, "top_access_cover", (0.100, 0.055, 0.006), (cover_x, 0.0, 0.028), cover_material)
    _box(part, "side_access_cover", (0.095, 0.006, 0.034), (cover_x + 0.030, -0.042, 0.0), cover_material)
    for ix, dx in enumerate((-0.037, 0.037)):
        for iy, dy in enumerate((-0.020, 0.020)):
            _cyl(part, f"top_cover_screw_{ix}_{iy}", 0.0042, 0.004, (cover_x + dx, dy, 0.033), bolt_material)
    for ix, dx in enumerate((-0.033, 0.033)):
        for iz, dz in enumerate((-0.011, 0.011)):
            _cyl(
                part,
                f"side_cover_screw_{ix}_{iz}",
                0.0037,
                0.004,
                (cover_x + 0.030 + dx, -0.046, dz),
                bolt_material,
                rpy=(math.pi / 2.0, 0.0, 0.0),
            )

    if terminal:
        _box(part, "terminal_block", (0.060, 0.090, 0.060), (length - 0.030, 0.0, 0.0), plate_material)
        _box(part, "terminal_face", (0.012, 0.110, 0.075), (length + 0.006, 0.0, 0.0), cover_material)
        for iy, dy in enumerate((-0.034, 0.034)):
            for iz, dz in enumerate((-0.020, 0.020)):
                _cyl(
                    part,
                    f"terminal_bolt_{iy}_{iz}",
                    0.0045,
                    0.004,
                    (length + 0.014, dy, dz),
                    bolt_material,
                    rpy=(0.0, math.pi / 2.0, 0.0),
                )
    else:
        _add_distal_yoke(part, length, plate_material, bearing_material, bolt_material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_pivot_box_section_chain")

    machined = model.material("dark_machined_steel", rgba=(0.19, 0.21, 0.22, 1.0))
    link_finish = model.material("blasted_link_faces", rgba=(0.34, 0.38, 0.40, 1.0))
    cover = model.material("black_access_covers", rgba=(0.025, 0.028, 0.030, 1.0))
    bearing = model.material("oiled_bearing_bronze", rgba=(0.72, 0.54, 0.25, 1.0))
    fastener = model.material("black_oxide_fasteners", rgba=(0.01, 0.01, 0.012, 1.0))
    datum = model.material("ground_datum_faces", rgba=(0.63, 0.65, 0.64, 1.0))

    base = model.part("base")
    _box(base, "base_plate", (0.310, 0.240, 0.026), (0.0, 0.0, 0.013), machined)
    _box(base, "front_mount_foot", (0.060, 0.070, 0.014), (0.098, 0.074, 0.033), datum)
    _box(base, "rear_mount_foot", (0.060, 0.070, 0.014), (-0.098, -0.074, 0.033), datum)
    _box(base, "cross_datum_rail", (0.250, 0.018, 0.018), (0.0, 0.0, 0.035), datum)
    _box(base, "pedestal", (0.110, 0.118, 0.118), (-0.018, 0.0, 0.085), machined)
    _box(base, "pedestal_front_rib", (0.020, 0.154, 0.090), (0.052, 0.0, 0.096), machined)
    _box(base, "pedestal_rear_rib", (0.020, 0.154, 0.090), (-0.086, 0.0, 0.096), machined)

    pivot_z = 0.180
    _box(base, "upper_yoke", (0.160, 0.160, 0.016), (0.0, 0.0, pivot_z + 0.045), machined)
    _box(base, "lower_yoke", (0.160, 0.160, 0.016), (0.0, 0.0, pivot_z - 0.045), machined)
    _box(base, "left_yoke_spacer", (0.056, 0.012, 0.092), (-0.020, 0.078, pivot_z), machined)
    _box(base, "right_yoke_spacer", (0.056, 0.012, 0.092), (-0.020, -0.078, pivot_z), machined)
    _box(base, "rear_yoke_web", (0.018, 0.130, 0.092), (-0.078, 0.0, pivot_z), machined)
    _cyl(base, "pin_shank", 0.018, 0.112, (0.0, 0.0, pivot_z), fastener)
    _cyl(base, "upper_boss", 0.070, 0.012, (0.0, 0.0, pivot_z + 0.059), bearing)
    _cyl(base, "lower_boss", 0.070, 0.012, (0.0, 0.0, pivot_z - 0.059), bearing)
    _cyl(base, "base_pin_head", 0.022, 0.008, (0.0, 0.0, pivot_z + 0.069), fastener)
    _add_bolt_pattern(base, "base_upper", 0.0, 0.050, pivot_z + 0.055, fastener, radius=0.0048, length=0.004)
    _add_bolt_pattern(base, "base_lower", 0.0, 0.050, pivot_z - 0.055, fastener, radius=0.0048, length=0.004)
    for ix, x in enumerate((-0.115, 0.115)):
        for iy, y in enumerate((-0.085, 0.085)):
            _cyl(base, f"mount_socket_{ix}_{iy}", 0.007, 0.005, (x, y, 0.0285), fastener)

    lengths = (0.330, 0.285, 0.250, 0.210)
    for idx, length in enumerate(lengths):
        link = model.part(f"link_{idx}")
        _add_box_section_link(
            link,
            length=length,
            body_material=link_finish,
            plate_material=machined,
            cover_material=cover,
            bearing_material=bearing,
            bolt_material=fastener,
            terminal=(idx == 3),
        )

    limits = MotionLimits(effort=95.0, velocity=1.7, lower=-1.20, upper=1.20)
    props = MotionProperties(damping=0.25, friction=0.08)
    model.articulation(
        "pivot_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child="link_0",
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
        motion_properties=props,
    )
    model.articulation(
        "pivot_1",
        ArticulationType.REVOLUTE,
        parent="link_0",
        child="link_1",
        origin=Origin(xyz=(lengths[0], 0.0, 0.0), rpy=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
        motion_properties=props,
    )
    model.articulation(
        "pivot_2",
        ArticulationType.REVOLUTE,
        parent="link_1",
        child="link_2",
        origin=Origin(xyz=(lengths[1], 0.0, 0.0), rpy=(0.0, 0.0, -1.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
        motion_properties=props,
    )
    model.articulation(
        "pivot_3",
        ArticulationType.REVOLUTE,
        parent="link_2",
        child="link_3",
        origin=Origin(xyz=(lengths[2], 0.0, 0.0), rpy=(0.0, 0.0, 0.78)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
        motion_properties=props,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [object_model.get_articulation(f"pivot_{i}") for i in range(4)]
    links = [object_model.get_part(f"link_{i}") for i in range(4)]
    base = object_model.get_part("base")

    pin_interfaces = [
        (base, links[0], "pin_shank", "pivot 0"),
        (links[0], links[1], "distal_pin_shank", "pivot 1"),
        (links[1], links[2], "distal_pin_shank", "pivot 2"),
        (links[2], links[3], "distal_pin_shank", "pivot 3"),
    ]
    for parent, child, pin_elem, label in pin_interfaces:
        for bearing_elem in ("proximal_hub", "hub_sleeve", "upper_bearing_cap", "lower_bearing_cap"):
            ctx.allow_overlap(
                parent,
                child,
                elem_a=pin_elem,
                elem_b=bearing_elem,
                reason=f"{label} uses a captured axle pin passing through simplified solid bearing hardware.",
            )
            ctx.expect_overlap(
                parent,
                child,
                axes="z",
                elem_a=pin_elem,
                elem_b=bearing_elem,
                min_overlap=0.004,
                name=f"{label} pin crosses {bearing_elem}",
            )
        ctx.expect_within(
            parent,
            child,
            axes="xy",
            inner_elem=pin_elem,
            outer_elem="proximal_hub",
            margin=0.001,
            name=f"{label} pin centered in hub",
        )

    ctx.check(
        "exactly four revolute joints",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "serial chain topology",
        [j.parent for j in object_model.articulations] == ["base", "link_0", "link_1", "link_2"]
        and [j.child for j in object_model.articulations] == ["link_0", "link_1", "link_2", "link_3"],
        details="expected base -> link_0 -> link_1 -> link_2 -> link_3",
    )

    # Each proximal hub is visibly captured in its parent's fork footprint while
    # retaining real vertical clearance to the upper and lower yoke plates.
    ctx.expect_within(links[0], base, axes="xy", inner_elem="proximal_hub", outer_elem="upper_yoke", name="base yoke surrounds first hub")
    ctx.expect_gap(base, links[0], axis="z", positive_elem="upper_yoke", negative_elem="upper_bearing_cap", min_gap=0.003, name="base upper bearing clearance")
    ctx.expect_gap(links[0], base, axis="z", positive_elem="lower_bearing_cap", negative_elem="lower_yoke", min_gap=0.003, name="base lower bearing clearance")

    for i in range(3):
        parent = links[i]
        child = links[i + 1]
        ctx.expect_within(child, parent, axes="xy", inner_elem="proximal_hub", outer_elem="distal_upper_yoke", name=f"pivot {i + 1} hub captured in fork")
        ctx.expect_gap(parent, child, axis="z", positive_elem="distal_upper_yoke", negative_elem="upper_bearing_cap", min_gap=0.003, name=f"pivot {i + 1} upper clearance")
        ctx.expect_gap(child, parent, axis="z", positive_elem="lower_bearing_cap", negative_elem="distal_lower_yoke", min_gap=0.003, name=f"pivot {i + 1} lower clearance")

    rest_tip = ctx.part_world_position(links[-1])
    with ctx.pose({joints[0]: 0.35, joints[1]: -0.45, joints[2]: 0.55, joints[3]: -0.35}):
        moved_tip = ctx.part_world_position(links[-1])
    ctx.check(
        "four pivots move the distal link",
        rest_tip is not None
        and moved_tip is not None
        and math.dist(rest_tip[:2], moved_tip[:2]) > 0.030,
        details=f"rest_tip={rest_tip}, moved_tip={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
