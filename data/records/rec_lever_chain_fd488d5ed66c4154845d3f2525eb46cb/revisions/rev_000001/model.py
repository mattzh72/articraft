from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _slot_cutter(x: float, y: float, length: float, width: float, depth: float):
    """CadQuery rounded slot cutter centered at (x, y) in the XY workplane."""
    r = width * 0.5
    slot = (
        cq.Workplane("XY")
        .center(x, y)
        .rect(max(length - width, 0.0001), width)
        .extrude(depth, both=True)
    )
    slot = slot.union(
        cq.Workplane("XY").center(x - (length - width) * 0.5, y).circle(r).extrude(depth, both=True)
    )
    slot = slot.union(
        cq.Workplane("XY").center(x + (length - width) * 0.5, y).circle(r).extrude(depth, both=True)
    )
    return slot


def _dogbone_plate(
    x_min: float,
    x_max: float,
    width: float,
    thickness: float,
    hole_positions: tuple[float, ...],
    *,
    hole_radius: float,
    slot_positions: tuple[float, ...] = (),
    slot_length: float = 0.060,
    slot_width: float = 0.012,
):
    """Flat rounded mechanical link plate in local XY, extruded through local Z."""
    length = x_max - x_min
    mid = (x_min + x_max) * 0.5
    body = cq.Workplane("XY").center(mid, 0.0).rect(length, width).extrude(thickness, both=True)
    body = body.union(cq.Workplane("XY").center(x_min, 0.0).circle(width * 0.5).extrude(thickness, both=True))
    body = body.union(cq.Workplane("XY").center(x_max, 0.0).circle(width * 0.5).extrude(thickness, both=True))
    for x in hole_positions:
        body = body.cut(
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(hole_radius)
            .extrude(thickness * 4.0, both=True)
        )
    for x in slot_positions:
        body = body.cut(_slot_cutter(x, 0.0, slot_length, slot_width, thickness * 4.0))
    return body


def _cover_plate(length: float, width: float, thickness: float):
    """Removable inspection plate with real slotted adjustment holes."""
    body = cq.Workplane("XY").rect(length, width).extrude(thickness, both=True)
    for x in (-length * 0.24, length * 0.24):
        body = body.cut(_slot_cutter(x, 0.0, 0.075, 0.016, thickness * 4.0))
    for x in (-length * 0.38, length * 0.38):
        for y in (-width * 0.30, width * 0.30):
            body = body.cut(cq.Workplane("XY").center(x, y).circle(0.006).extrude(thickness * 4.0, both=True))
    return body


def _endpoint(pivot: tuple[float, float, float], local_x: float, pitch: float) -> tuple[float, float, float]:
    """Endpoint of a local X offset after the lever's rest pitch about Y."""
    return (
        pivot[0] + local_x * math.cos(pitch),
        pivot[1],
        pivot[2] - local_x * math.sin(pitch),
    )


def _pitch_between(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = b[0] - a[0]
    dz = b[2] - a[2]
    return -math.atan2(dz, dx)


def _distance_xz(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.hypot(b[0] - a[0], b[2] - a[2])


def _cyl_y(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="staggered_lever_chain_study")

    pi = math.pi
    steel = Material("bead_blasted_steel", rgba=(0.45, 0.48, 0.50, 1.0))
    dark_steel = Material("dark_oxide_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    arm_blue = Material("tempered_blue_steel", rgba=(0.20, 0.30, 0.42, 1.0))
    cover_gray = Material("removable_cover_gray", rgba=(0.36, 0.38, 0.39, 1.0))
    bronze = Material("oilite_bronze_bushings", rgba=(0.72, 0.52, 0.26, 1.0))
    black = Material("black_recess_shadow", rgba=(0.02, 0.02, 0.018, 1.0))
    rubber = Material("matte_black_stop_pads", rgba=(0.01, 0.011, 0.010, 1.0))

    # Kinematic station layout in meters.  Motion lies in the X/Z plane and
    # all primary pivot axes run on Y, as on an exposed bench linkage.
    pivots = {
        "input_lever": (-0.55, 0.0, 0.130),
        "rocker_0": (-0.18, 0.0, 0.220),
        "rocker_1": (0.24, 0.0, 0.155),
        "output_lever": (0.68, 0.0, 0.205),
    }
    pitches = {
        "input_lever": math.radians(-35.0),
        "rocker_0": math.radians(-7.0),
        "rocker_1": math.radians(12.0),
        "output_lever": math.radians(-20.0),
    }
    link_extents = {
        "input_lever": (0.0, 0.20),
        "rocker_0": (-0.12, 0.14),
        "rocker_1": (-0.16, 0.16),
        "output_lever": (-0.20, 0.22),
    }
    input_end = _endpoint(pivots["input_lever"], link_extents["input_lever"][1], pitches["input_lever"])
    rocker0_in = _endpoint(pivots["rocker_0"], link_extents["rocker_0"][0], pitches["rocker_0"])
    rocker0_out = _endpoint(pivots["rocker_0"], link_extents["rocker_0"][1], pitches["rocker_0"])
    rocker1_in = _endpoint(pivots["rocker_1"], link_extents["rocker_1"][0], pitches["rocker_1"])
    rocker1_out = _endpoint(pivots["rocker_1"], link_extents["rocker_1"][1], pitches["rocker_1"])
    output_in = _endpoint(pivots["output_lever"], link_extents["output_lever"][0], pitches["output_lever"])

    base = model.part("base")
    base.visual(Box((1.90, 0.36, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.0125)), material=dark_steel, name="ground_plate")
    for y in (-0.155, 0.155):
        base.visual(Box((1.78, 0.028, 0.028)), origin=Origin(xyz=(0.0, y, 0.039)), material=steel, name=f"side_rail_{y:+.3f}")
        for x in (-0.62, -0.36, -0.10, 0.16, 0.42, 0.68):
            base.visual(Box((0.058, 0.010, 0.002)), origin=Origin(xyz=(x, y, 0.0535)), material=black, name=f"rail_slot_{x:+.2f}_{y:+.2f}")
            base.visual(Cylinder(radius=0.005, length=0.0025), origin=Origin(xyz=(x - 0.029, y, 0.054), rpy=(0.0, 0.0, 0.0)), material=black, name=f"slot_end_a_{x:+.2f}_{y:+.2f}")
            base.visual(Cylinder(radius=0.005, length=0.0025), origin=Origin(xyz=(x + 0.029, y, 0.054), rpy=(0.0, 0.0, 0.0)), material=black, name=f"slot_end_b_{x:+.2f}_{y:+.2f}")
    for x in (p[0] for p in pivots.values()):
        base.visual(Box((0.045, 0.315, 0.018)), origin=Origin(xyz=(x, 0.0, 0.034)), material=steel, name=f"cross_tie_{x:+.2f}")

    # Four fabricated yoke brackets.  They are intentionally open so the moving
    # lever plates and washers can be seen between the cheek plates.
    for name_key, pivot in pivots.items():
        height = (pivot[2] - 0.025) + 0.075
        cheek_z = 0.024 + height * 0.5
        for y in (-0.050, 0.050):
            base.visual(Box((0.086, 0.014, height)), origin=Origin(xyz=(pivot[0], y, cheek_z)), material=steel, name=f"{name_key}_cheek_{y:+.2f}")
            base.visual(
                Cylinder(radius=0.027, length=0.012),
                origin=Origin(xyz=(pivot[0], y * 1.23, pivot[2]), rpy=(pi / 2.0, 0.0, 0.0)),
                material=bronze,
                name=f"{name_key}_outer_bushing_{y:+.2f}",
            )
            base.visual(
                Cylinder(radius=0.010, length=0.002),
                origin=Origin(xyz=(pivot[0], y * 1.37, pivot[2]), rpy=(pi / 2.0, 0.0, 0.0)),
                material=black,
                name=f"{name_key}_bore_shadow_{y:+.2f}",
            )
        base.visual(Box((0.018, 0.112, min(0.090, height * 0.55))), origin=Origin(xyz=(pivot[0] - 0.056, 0.0, 0.066)), material=steel, name=f"{name_key}_rear_web")
        for bx in (pivot[0] - 0.027, pivot[0] + 0.027):
            for by in (-0.073, 0.073):
                base.visual(Cylinder(radius=0.006, length=0.006), origin=Origin(xyz=(bx, by, 0.028)), material=steel, name=f"{name_key}_base_bolt_{bx:+.2f}_{by:+.2f}")

    # Rubber/urethane stop pads show the angular travel limits without a closed
    # decorative housing.
    stop_specs = [
        (-0.42, -0.095, 0.034, 12.0),
        (-0.05, 0.105, 0.034, -10.0),
        (0.36, -0.100, 0.034, 16.0),
        (0.78, 0.095, 0.034, -14.0),
    ]
    for i, (x, y, z, yaw_deg) in enumerate(stop_specs):
        base.visual(
            Box((0.070, 0.026, 0.018)),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, math.radians(yaw_deg))),
            material=rubber,
            name=f"stop_pad_{i}",
        )

    # Standoffs for removable access covers.
    for cover_x in (-0.36, 0.36):
        for sx in (-0.105, 0.105):
            for sy in (-0.030, 0.030):
                base.visual(Cylinder(radius=0.010, length=0.056), origin=Origin(xyz=(cover_x + sx, -0.110 + sy, 0.053)), material=steel, name=f"cover_standoff_{cover_x:+.2f}_{sx:+.2f}_{sy:+.2f}")

    def add_lever_part(part_name: str, x_min: float, x_max: float, width: float, source_holes: tuple[float, ...]):
        part = model.part(part_name)
        plate = _dogbone_plate(
            x_min,
            x_max,
            width,
            0.018,
            source_holes,
            hole_radius=0.014,
            slot_positions=(0.5 * (x_min + x_max),),
            slot_length=min(0.090, (x_max - x_min) * 0.42),
            slot_width=0.014,
        )
        part.visual(
            mesh_from_cadquery(plate, f"{part_name}_plate", tolerance=0.0008, angular_tolerance=0.08),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=arm_blue,
            name="web_plate",
        )
        for local_x in source_holes:
            # Bronze bearing insert through the lever and two bright spacer washers.
            part.visual(Cylinder(radius=0.022, length=0.086), origin=Origin(xyz=(local_x, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=bronze, name=f"bearing_{local_x:+.2f}")
            for y in (-0.012, 0.012):
                part.visual(Cylinder(radius=0.026, length=0.006), origin=Origin(xyz=(local_x, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name=f"washer_{local_x:+.2f}_{y:+.2f}")
            # Outboard drive pin for coupler/follower hardware.  It is sized to
            # just kiss the coupler bore so the standalone study has a visible
            # physical support path without broad interpenetration.
            if abs(local_x) > 1e-6:
                part.visual(Cylinder(radius=0.012, length=0.078), origin=Origin(xyz=(local_x, 0.041, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name=f"outboard_pin_{local_x:+.2f}")
                part.visual(Cylinder(radius=0.016, length=0.006), origin=Origin(xyz=(local_x, 0.083, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name=f"pin_cap_{local_x:+.2f}")
        return part

    input_lever = add_lever_part("input_lever", 0.0, 0.20, 0.070, (0.0, 0.20))
    rocker_0 = add_lever_part("rocker_0", -0.12, 0.14, 0.068, (-0.12, 0.0, 0.14))
    rocker_1 = add_lever_part("rocker_1", -0.16, 0.16, 0.066, (-0.16, 0.0, 0.16))
    output_lever = add_lever_part("output_lever", -0.20, 0.22, 0.070, (-0.20, 0.0, 0.22))

    # Separate dogbone couplers running on the front side of the mechanism.
    def add_coupler(name: str, length: float):
        part = model.part(name)
        shape = _dogbone_plate(0.0, length, 0.034, 0.010, (0.0, length), hole_radius=0.012)
        part.visual(mesh_from_cadquery(shape, f"{name}_plate", tolerance=0.0008, angular_tolerance=0.08), origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name="link_plate")
        return part

    c0_len = _distance_xz(input_end, rocker0_in)
    c1_len = _distance_xz(rocker0_out, rocker1_in)
    c2_len = _distance_xz(rocker1_out, output_in)
    coupler_0 = add_coupler("coupler_0", c0_len)
    coupler_1 = add_coupler("coupler_1", c1_len)
    coupler_2 = add_coupler("coupler_2", c2_len)

    # Removable slotted access covers.
    for idx, cover_x in enumerate((-0.36, 0.36)):
        cover = model.part(f"access_cover_{idx}")
        cover.visual(
            mesh_from_cadquery(_cover_plate(0.280, 0.090, 0.006), f"access_cover_{idx}_panel", tolerance=0.0008, angular_tolerance=0.08),
            origin=Origin(),
            material=cover_gray,
            name="slotted_panel",
        )
        for sx in (-0.105, 0.105):
            for sy in (-0.030, 0.030):
                cover.visual(Cylinder(radius=0.0075, length=0.005), origin=Origin(xyz=(sx, sy, 0.0055)), material=steel, name=f"captive_screw_{sx:+.2f}_{sy:+.2f}")
        model.articulation(
            f"base_to_access_cover_{idx}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=cover,
            origin=Origin(xyz=(cover_x, -0.110, 0.084)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=0.08, lower=0.0, upper=0.060),
        )

    # Primary bracket-mounted lever pivots.
    input_joint = model.articulation(
        "base_to_input_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=input_lever,
        origin=Origin(xyz=pivots["input_lever"], rpy=(0.0, pitches["input_lever"], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "base_to_rocker_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker_0,
        origin=Origin(xyz=pivots["rocker_0"], rpy=(0.0, pitches["rocker_0"], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=-0.45, upper=0.45),
        mimic=Mimic(joint="base_to_input_lever", multiplier=-0.65, offset=0.0),
    )
    model.articulation(
        "base_to_rocker_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker_1,
        origin=Origin(xyz=pivots["rocker_1"], rpy=(0.0, pitches["rocker_1"], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=-0.45, upper=0.45),
        mimic=Mimic(joint="base_to_input_lever", multiplier=0.55, offset=0.0),
    )
    model.articulation(
        "base_to_output_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=output_lever,
        origin=Origin(xyz=pivots["output_lever"], rpy=(0.0, pitches["output_lever"], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=75.0, velocity=1.2, lower=-0.55, upper=0.55),
        mimic=Mimic(joint="base_to_input_lever", multiplier=-0.75, offset=0.0),
    )

    # Coupler joints: each front-side dogbone is pinned to the previous lever's
    # outboard drive pin and visually bears on the next lever's follower tab.
    model.articulation(
        "input_lever_to_coupler_0",
        ArticulationType.REVOLUTE,
        parent=input_lever,
        child=coupler_0,
        origin=Origin(xyz=(0.20, 0.065, 0.0), rpy=(0.0, _pitch_between(input_end, rocker0_in) - pitches["input_lever"], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.70, upper=0.70),
        mimic=Mimic(joint="base_to_input_lever", multiplier=0.22, offset=0.0),
    )
    model.articulation(
        "rocker_0_to_coupler_1",
        ArticulationType.REVOLUTE,
        parent=rocker_0,
        child=coupler_1,
        origin=Origin(xyz=(0.14, 0.065, 0.0), rpy=(0.0, _pitch_between(rocker0_out, rocker1_in) - pitches["rocker_0"], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.70, upper=0.70),
        mimic=Mimic(joint="base_to_input_lever", multiplier=-0.18, offset=0.0),
    )
    model.articulation(
        "rocker_1_to_coupler_2",
        ArticulationType.REVOLUTE,
        parent=rocker_1,
        child=coupler_2,
        origin=Origin(xyz=(0.16, 0.065, 0.0), rpy=(0.0, _pitch_between(rocker1_out, output_in) - pitches["rocker_1"], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.70, upper=0.70),
        mimic=Mimic(joint="base_to_input_lever", multiplier=0.20, offset=0.0),
    )

    # Store a few design locations for tests without hard-coding duplicate math.
    model.meta["pivot_count"] = 4
    model.meta["coupler_count"] = 3
    model.meta["input_joint_name"] = input_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    input_joint = object_model.get_articulation("base_to_input_lever")
    output_lever = object_model.get_part("output_lever")
    input_lever = object_model.get_part("input_lever")

    def elem_center_z(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[2] + hi[2])

    ctx.check("four bracketed lever pivots", object_model.meta.get("pivot_count") == 4)
    ctx.check("three front coupler links", object_model.meta.get("coupler_count") == 3)
    captured_pin_pairs = (
        ("coupler_0", "input_lever", "outboard_pin_+0.20"),
        ("coupler_0", "rocker_0", "outboard_pin_-0.12"),
        ("coupler_1", "rocker_0", "outboard_pin_+0.14"),
        ("coupler_1", "rocker_1", "outboard_pin_-0.16"),
        ("coupler_2", "rocker_1", "outboard_pin_+0.16"),
        ("coupler_2", "output_lever", "outboard_pin_-0.20"),
    )
    for coupler_name, lever_name, pin_elem in captured_pin_pairs:
        ctx.allow_overlap(
            coupler_name,
            lever_name,
            elem_a="link_plate",
            elem_b=pin_elem,
            reason="The coupler bore is represented as a captured pin fit with a solid pin proxy through the dogbone link.",
        )
        ctx.expect_overlap(
            coupler_name,
            lever_name,
            axes="xz",
            elem_a="link_plate",
            elem_b=pin_elem,
            min_overlap=0.010,
            name=f"{coupler_name} retained on {pin_elem}",
        )
    ctx.expect_overlap("input_lever", "base", axes="xy", min_overlap=0.03, elem_a="web_plate", name="input lever is centered in its yoke footprint")
    ctx.expect_overlap("rocker_0", "base", axes="xy", min_overlap=0.03, elem_a="web_plate", name="first rocker is centered in its yoke footprint")
    ctx.expect_gap("access_cover_0", "base", axis="z", positive_elem="slotted_panel", negative_elem="ground_plate", max_gap=0.070, max_penetration=0.0, name="cover zero pose remains above ground plate")

    rest_output = elem_center_z(output_lever, "pin_cap_+0.22")
    rest_input = elem_center_z(input_lever, "pin_cap_+0.20")
    with ctx.pose({input_joint: 0.30}):
        moved_output = elem_center_z(output_lever, "pin_cap_+0.22")
        moved_input = elem_center_z(input_lever, "pin_cap_+0.20")
        ctx.expect_gap("access_cover_1", "base", axis="z", positive_elem="slotted_panel", negative_elem="ground_plate", max_gap=0.070, max_penetration=0.0, name="second cover stays removable during linkage pose")
    ctx.check(
        "input and output levers both respond to drive joint",
        rest_output is not None
        and moved_output is not None
        and rest_input is not None
        and moved_input is not None
        and abs(moved_output - rest_output) > 0.010
        and abs(moved_input - rest_input) > 0.010,
        details=f"input rest/moved={rest_input}/{moved_input}, output rest/moved={rest_output}/{moved_output}",
    )
    return ctx.report()


object_model = build_object_model()
