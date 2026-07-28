from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

TARGET_OBJECT = "__TARGET_OBJECT__"
PI = math.pi


def materials(model: ArticulatedObject) -> dict[str, object]:
    colors = {
        "dark": (0.055, 0.065, 0.075, 1.0),
        "black": (0.015, 0.018, 0.022, 1.0),
        "steel": (0.52, 0.58, 0.62, 1.0),
        "silver": (0.78, 0.82, 0.84, 1.0),
        "glass": (0.12, 0.28, 0.34, 0.65),
        "orange": (0.92, 0.36, 0.055, 1.0),
        "yellow": (0.92, 0.63, 0.08, 1.0),
        "blue": (0.055, 0.18, 0.38, 1.0),
        "green": (0.18, 0.38, 0.16, 1.0),
        "cream": (0.76, 0.74, 0.62, 1.0),
        "white": (0.84, 0.86, 0.86, 1.0),
        "red": (0.72, 0.055, 0.035, 1.0),
        "wood": (0.29, 0.12, 0.045, 1.0),
        "fabric": (0.2, 0.35, 0.42, 1.0),
        "rubber": (0.025, 0.028, 0.03, 1.0),
    }
    return {name: model.material(name, rgba=rgba) for name, rgba in colors.items()}


def box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def sphere(part, name, radius, xyz, material) -> None:
    part.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)


def fixed(model, name, parent, child, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> None:
    model.articulation(
        name,
        ArticulationType.FIXED,
        parent=parent,
        child=child,
        origin=Origin(xyz=xyz, rpy=rpy),
    )


def revolute(model, name, parent, child, xyz, axis, lower, upper, effort=250.0) -> None:
    model.articulation(
        name,
        ArticulationType.REVOLUTE,
        parent=parent,
        child=child,
        origin=Origin(xyz=xyz),
        axis=axis,
        motion_limits=MotionLimits(
            effort=effort,
            velocity=1.2,
            lower=lower,
            upper=upper,
        ),
    )


def prismatic(model, name, parent, child, xyz, axis, lower, upper, effort=250.0) -> None:
    model.articulation(
        name,
        ArticulationType.PRISMATIC,
        parent=parent,
        child=child,
        origin=Origin(xyz=xyz),
        axis=axis,
        motion_limits=MotionLimits(
            effort=effort,
            velocity=0.3,
            lower=lower,
            upper=upper,
        ),
    )


def continuous(model, name, parent, child, xyz, axis, effort=100.0) -> None:
    model.articulation(
        name,
        ArticulationType.CONTINUOUS,
        parent=parent,
        child=child,
        origin=Origin(xyz=xyz),
        axis=axis,
        motion_limits=MotionLimits(effort=effort, velocity=4.0),
    )


def wheel(part, material, metal, radius, width, axis="y") -> None:
    rpy = (PI / 2, 0, 0) if axis == "y" else (0, PI / 2, 0)
    cyl(part, "tire", radius, width, (0, 0, 0), material, rpy)
    cyl(part, "rim", radius * 0.61, width * 1.04, (0, 0, 0), metal, rpy)
    cyl(part, "hub", radius * 0.2, width * 1.18, (0, 0, 0), material, rpy)
    for index in range(8):
        angle = index * PI / 4
        box(
            part,
            f"spoke_{index}",
            (radius * 0.72, width * 0.08, radius * 0.035),
            (math.cos(angle) * radius * 0.27, 0, math.sin(angle) * radius * 0.27),
            metal,
            (0, -angle, 0),
        )


def build_compact_excavator() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_compact_excavator")
    m = materials(model)
    base = model.part("undercarriage")
    box(base, "center_frame", (1.25, 0.66, 0.18), (0, 0, 0.42), m["dark"])
    for side in (-1, 1):
        y = side * 0.43
        box(base, f"track_shell_{side}", (1.4, 0.27, 0.42), (0, y, 0.28), m["rubber"])
        for index, x in enumerate((-0.48, -0.24, 0, 0.24, 0.48)):
            cyl(
                base,
                f"track_roller_{side}_{index}",
                0.14,
                0.3,
                (x, y, 0.28),
                m["steel"],
                (PI / 2, 0, 0),
            )
        box(base, f"track_pad_{side}", (1.38, 0.3, 0.07), (0, y, 0.08), m["dark"])
    cyl(base, "swing_bearing", 0.31, 0.14, (0, 0, 0.57), m["steel"])
    upper = model.part("upper_body")
    box(upper, "deck", (1.0, 0.78, 0.14), (0, 0, 0.07), m["orange"])
    box(upper, "counterweight", (0.55, 0.72, 0.46), (-0.25, 0, 0.35), m["orange"])
    box(upper, "engine_hood", (0.58, 0.7, 0.24), (-0.2, 0, 0.68), m["orange"])
    box(upper, "cab_floor", (0.4, 0.64, 0.08), (0.27, 0, 0.24), m["dark"])
    for y in (-0.28, 0.28):
        box(upper, f"cab_pillar_{y}", (0.06, 0.06, 0.66), (0.37, y, 0.58), m["dark"])
        box(upper, f"cab_window_{y}", (0.32, 0.018, 0.46), (0.2, y, 0.58), m["glass"])
    box(upper, "cab_roof", (0.48, 0.66, 0.08), (0.2, 0, 0.94), m["orange"])
    for index, z in enumerate((0.34, 0.44, 0.54)):
        box(upper, f"engine_vent_{index}", (0.28, 0.018, 0.035), (-0.28, -0.365, z), m["black"])
    continuous(model, "body_swing", base, upper, (0, 0, 0.63), (0, 0, 1), 900)
    blade = model.part("dozer_blade")
    box(blade, "blade", (0.18, 1.08, 0.42), (0.08, 0, -0.12), m["orange"])
    box(blade, "cutting_edge", (0.24, 1.16, 0.07), (0.12, 0, -0.34), m["steel"])
    for y in (-0.35, 0.35):
        box(blade, f"lift_arm_{y}", (0.42, 0.1, 0.1), (-0.18, y, -0.02), m["dark"])
    revolute(model, "blade_lift", base, blade, (0.68, 0, 0.4), (0, 1, 0), -0.35, 0.5)
    boom = model.part("boom")
    box(boom, "boom_member", (0.92, 0.22, 0.22), (0.44, 0, 0.08), m["orange"], (0, -0.18, 0))
    cyl(boom, "pivot_boss", 0.13, 0.28, (0, 0, 0), m["dark"], (PI / 2, 0, 0))
    cyl(boom, "hydraulic_barrel", 0.045, 0.56, (0.32, 0, 0.23), m["dark"], (0, PI / 2, 0))
    cyl(boom, "hydraulic_rod", 0.023, 0.34, (0.72, 0, 0.23), m["silver"], (0, PI / 2, 0))
    revolute(model, "boom_raise", upper, boom, (0.48, 0, 0.62), (0, -1, 0), -0.6, 1.0, 800)
    stick = model.part("stick")
    box(stick, "stick_member", (0.72, 0.18, 0.2), (0.34, 0, -0.05), m["orange"], (0, 0.2, 0))
    cyl(stick, "pivot_boss", 0.1, 0.25, (0, 0, 0), m["dark"], (PI / 2, 0, 0))
    cyl(stick, "bucket_ram", 0.034, 0.42, (0.3, 0, 0.12), m["dark"], (0, PI / 2, 0))
    revolute(model, "stick_curl", boom, stick, (0.88, 0, -0.08), (0, -1, 0), -1.7, 0.45, 600)
    bucket = model.part("bucket")
    box(bucket, "bucket_shell", (0.4, 0.46, 0.3), (0.17, 0, -0.1), m["orange"])
    box(bucket, "cutting_edge", (0.15, 0.52, 0.08), (0.4, 0, -0.22), m["steel"])
    for y in (-0.18, -0.06, 0.06, 0.18):
        box(bucket, f"tooth_{y}", (0.18, 0.07, 0.07), (0.49, y, -0.25), m["steel"])
    revolute(model, "bucket_curl", stick, bucket, (0.68, 0, -0.1), (0, -1, 0), -1.9, 0.5, 500)
    door = model.part("cab_door")
    box(door, "frame", (0.34, 0.035, 0.58), (0.16, 0, 0.29), m["dark"])
    box(door, "window", (0.27, 0.012, 0.42), (0.16, -0.024, 0.33), m["glass"])
    box(door, "handle", (0.1, 0.03, 0.025), (0.28, -0.04, 0.2), m["black"])
    revolute(model, "cab_door_hinge", upper, door, (0.37, -0.3, 0.3), (0, 0, 1), 0, 1.35, 30)
    return model


def build_powered_hospital_bed() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_powered_hospital_bed")
    m = materials(model)
    base = model.part("chassis")
    box(base, "frame", (1.55, 0.68, 0.12), (0, 0, 0.32), m["dark"])
    box(base, "battery", (0.34, 0.3, 0.18), (0.45, 0, 0.43), m["white"])
    for index, (x, y) in enumerate(((-0.66, -0.4), (-0.66, 0.4), (0.66, -0.4), (0.66, 0.4))):
        box(base, f"caster_fork_{index}", (0.1, 0.12, 0.2), (x, y, 0.2), m["steel"])
        wheel_part = model.part(f"caster_wheel_{index}")
        wheel(wheel_part, m["rubber"], m["silver"], 0.11, 0.09)
        continuous(model, f"caster_spin_{index}", base, wheel_part, (x, y, 0.1), (0, 1, 0), 30)
    lift = model.part("lift_column")
    box(lift, "inner_column", (0.3, 0.3, 0.7), (0, 0, 0.35), m["steel"])
    box(lift, "column_cap", (0.42, 0.4, 0.08), (0, 0, 0.72), m["dark"])
    cyl(lift, "actuator", 0.05, 0.62, (0.14, 0, 0.34), m["silver"])
    prismatic(model, "bed_height", base, lift, (0, 0, 0.4), (0, 0, 1), 0, 0.42, 1200)
    platform = model.part("platform")
    box(platform, "deck", (2.05, 0.94, 0.09), (0, 0, 0.045), m["white"])
    box(platform, "seat_cushion", (0.82, 0.86, 0.16), (0, 0, 0.16), m["fabric"])
    for x in (-0.92, 0.92):
        box(platform, f"bumper_{x}", (0.09, 1.0, 0.2), (x, 0, 0.13), m["white"])
    fixed(model, "lift_to_platform", lift, platform, (0, 0, 0.78))
    back = model.part("backrest")
    box(back, "panel", (0.72, 0.86, 0.08), (-0.34, 0, 0.04), m["white"])
    box(back, "cushion", (0.66, 0.8, 0.15), (-0.34, 0, 0.15), m["fabric"])
    cyl(back, "hinge_barrel", 0.045, 0.9, (0, 0, 0), m["steel"], (PI / 2, 0, 0))
    revolute(model, "backrest_raise", platform, back, (-0.41, 0, 0.1), (0, -1, 0), 0, 1.25, 500)
    leg = model.part("leg_section")
    box(leg, "panel", (0.7, 0.86, 0.08), (0.33, 0, 0.04), m["white"])
    box(leg, "cushion", (0.64, 0.8, 0.15), (0.33, 0, 0.15), m["fabric"])
    revolute(model, "leg_raise", platform, leg, (0.41, 0, 0.1), (0, 1, 0), -0.3, 0.65, 450)
    for index, y in enumerate((-0.49, 0.49)):
        rail = model.part(f"side_rail_{index}")
        box(rail, "top_rail", (0.72, 0.045, 0.05), (0, 0, 0.42), m["steel"])
        for x in (-0.32, 0, 0.32):
            box(rail, f"post_{x}", (0.04, 0.05, 0.42), (x, 0, 0.21), m["steel"])
        cyl(rail, "hinge", 0.035, 0.78, (0, 0, 0), m["dark"], (0, PI / 2, 0))
        revolute(
            model, f"rail_fold_{index}", platform, rail, (0, y, 0.15), (1, 0, 0), -1.45, 0, 100
        )
    return model


def build_folding_bicycle() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_folding_bicycle")
    m = materials(model)
    rear = model.part("rear_frame")
    for name, size, xyz, rpy in (
        ("chainstay_0", (0.55, 0.035, 0.04), (-0.28, -0.07, 0.35), (0, -0.12, 0)),
        ("chainstay_1", (0.55, 0.035, 0.04), (-0.28, 0.07, 0.35), (0, -0.12, 0)),
        ("seat_tube", (0.06, 0.08, 0.56), (-0.05, 0, 0.58), (0, -0.22, 0)),
        ("top_brace", (0.5, 0.05, 0.05), (-0.22, 0, 0.66), (0, 0.45, 0)),
    ):
        box(rear, name, size, xyz, m["blue"], rpy)
    cyl(rear, "fold_hinge", 0.075, 0.14, (0.05, 0, 0.58), m["silver"], (PI / 2, 0, 0))
    rear_wheel = model.part("rear_wheel")
    wheel(rear_wheel, m["rubber"], m["silver"], 0.31, 0.075)
    continuous(model, "rear_wheel_spin", rear, rear_wheel, (-0.55, 0, 0.31), (0, 1, 0), 30)
    front = model.part("front_frame")
    box(front, "top_tube", (0.55, 0.06, 0.06), (0.27, 0, 0.06), m["blue"], (0, 0.15, 0))
    box(front, "down_tube", (0.58, 0.07, 0.07), (0.28, 0, -0.12), m["blue"], (0, -0.45, 0))
    cyl(front, "hinge_boss", 0.075, 0.15, (0, 0, 0), m["dark"], (PI / 2, 0, 0))
    box(front, "latch", (0.08, 0.16, 0.05), (0.06, 0, 0.02), m["orange"])
    revolute(model, "frame_fold", rear, front, (0.05, 0, 0.58), (0, 0, 1), 0, 2.8, 300)
    fork = model.part("front_fork")
    box(fork, "steerer", (0.07, 0.08, 0.38), (0, 0, -0.16), m["dark"], (0, 0.25, 0))
    for y in (-0.08, 0.08):
        box(fork, f"fork_blade_{y}", (0.06, 0.04, 0.45), (0.12, y, -0.48), m["blue"], (0, -0.5, 0))
    box(fork, "crown", (0.18, 0.2, 0.06), (0.04, 0, -0.28), m["dark"])
    revolute(model, "steering", front, fork, (0.52, 0, 0.05), (0, 0, 1), -0.85, 0.85, 80)
    front_wheel = model.part("front_wheel")
    wheel(front_wheel, m["rubber"], m["silver"], 0.31, 0.075)
    continuous(model, "front_wheel_spin", fork, front_wheel, (0.24, 0, -0.66), (0, 1, 0), 30)
    crank = model.part("crank")
    cyl(crank, "chainring", 0.11, 0.02, (0, 0.09, 0), m["steel"], (PI / 2, 0, 0))
    for sign in (-1, 1):
        box(
            crank,
            f"arm_{sign}",
            (0.18, 0.025, 0.025),
            (sign * 0.08, sign * 0.1, 0),
            m["dark"],
            (0, sign * 0.4, 0),
        )
        box(
            crank,
            f"pedal_{sign}",
            (0.09, 0.04, 0.025),
            (sign * 0.18, sign * 0.1, sign * 0.04),
            m["rubber"],
        )
    continuous(model, "crank_spin", rear, crank, (-0.02, 0, 0.33), (0, 1, 0), 40)
    seat = model.part("seat_post")
    cyl(seat, "post", 0.022, 0.55, (0, 0, 0.1), m["silver"])
    box(seat, "saddle", (0.23, 0.1, 0.05), (-0.05, 0, 0.39), m["black"])
    prismatic(model, "seat_height", rear, seat, (-0.1, 0, 0.63), (0, 0, 1), 0, 0.2, 80)
    stem = model.part("handlebar_stem")
    box(stem, "stem", (0.06, 0.06, 0.54), (0, 0, 0.24), m["silver"], (0, -0.16, 0))
    box(stem, "bar", (0.06, 0.52, 0.05), (-0.04, 0, 0.51), m["dark"])
    for y in (-0.24, 0.24):
        box(stem, f"grip_{y}", (0.08, 0.1, 0.07), (-0.04, y, 0.51), m["rubber"])
    revolute(model, "stem_fold", fork, stem, (0, 0, 0.02), (1, 0, 0), 0, 1.55, 80)
    return model


def build_sliding_compound_miter_saw() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_sliding_compound_miter_saw")
    m = materials(model)
    base = model.part("base")
    box(base, "base_plate", (0.78, 0.62, 0.09), (0, 0, 0.045), m["dark"])
    for x in (-0.3, 0.3):
        box(base, f"wing_{x}", (0.2, 0.48, 0.06), (x, 0, 0.1), m["steel"])
    table = model.part("rotary_table")
    cyl(table, "table_disk", 0.28, 0.06, (0, 0, 0.03), m["steel"])
    box(table, "fence", (0.68, 0.05, 0.16), (0, 0.18, 0.12), m["silver"])
    box(table, "throat", (0.06, 0.42, 0.025), (0, -0.02, 0.07), m["black"])
    for index, x in enumerate((-0.2, -0.14, -0.08, 0.08, 0.14, 0.2)):
        box(table, f"scale_tick_{index}", (0.012, 0.05, 0.01), (x, -0.23, 0.065), m["black"])
    revolute(model, "miter_angle", base, table, (0, 0, 0.09), (0, 0, 1), -0.85, 0.85, 120)
    rails = model.part("rail_carriage")
    for x in (-0.09, 0.09):
        cyl(rails, f"rail_{x}", 0.02, 0.58, (x, 0, 0.38), m["silver"], (PI / 2, 0, 0))
    box(rails, "rear_bridge", (0.3, 0.1, 0.56), (0, 0.27, 0.24), m["dark"])
    box(rails, "carriage", (0.32, 0.12, 0.16), (0, 0.02, 0.38), m["orange"])
    prismatic(model, "head_slide", table, rails, (0, 0.16, 0.08), (0, -1, 0), 0, 0.36, 350)
    bevel = model.part("bevel_head")
    box(bevel, "yoke", (0.23, 0.12, 0.14), (0, 0, 0.07), m["dark"])
    cyl(bevel, "pivot", 0.05, 0.28, (0, 0, 0), m["steel"], (PI / 2, 0, 0))
    revolute(model, "bevel_angle", rails, bevel, (0, -0.08, 0.42), (1, 0, 0), -0.8, 0.8, 300)
    arm = model.part("saw_arm")
    box(arm, "arm_spine", (0.16, 0.48, 0.1), (0, -0.22, -0.02), m["dark"])
    box(arm, "motor", (0.26, 0.24, 0.2), (0, -0.36, 0.08), m["orange"])
    for index, z in enumerate((0.03, 0.09, 0.15)):
        box(arm, f"motor_vent_{index}", (0.27, 0.03, 0.018), (0, -0.49, z), m["black"])
    box(arm, "handle", (0.12, 0.32, 0.08), (0, -0.48, 0.23), m["black"])
    revolute(model, "saw_plunge", bevel, arm, (0, 0, 0.1), (1, 0, 0), -0.15, 1.15, 300)
    blade = model.part("blade")
    cyl(blade, "blade_disc", 0.17, 0.012, (0, 0, 0), m["silver"], (PI / 2, 0, 0))
    cyl(blade, "hub", 0.04, 0.04, (0, 0, 0), m["dark"], (PI / 2, 0, 0))
    continuous(model, "blade_spin", arm, blade, (0, -0.44, -0.06), (0, 1, 0), 80)
    guard = model.part("blade_guard")
    cyl(guard, "guard_shell", 0.19, 0.045, (0, 0, 0), m["orange"], (PI / 2, 0, 0))
    box(guard, "guard_cutaway", (0.2, 0.06, 0.2), (0.08, 0, -0.08), m["dark"])
    revolute(model, "guard_retract", arm, guard, (0, -0.44, -0.06), (0, 1, 0), 0, 1.1, 30)
    return model


def build_dishwasher() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_dishwasher")
    m = materials(model)
    cabinet = model.part("cabinet")
    box(cabinet, "back", (0.58, 0.06, 0.82), (0, 0.29, 0.43), m["steel"])
    for x in (-0.3, 0.3):
        box(cabinet, f"side_{x}", (0.04, 0.62, 0.86), (x, 0, 0.43), m["silver"])
    box(cabinet, "top", (0.64, 0.62, 0.05), (0, 0, 0.86), m["silver"])
    box(cabinet, "bottom", (0.64, 0.62, 0.06), (0, 0, 0.03), m["silver"])
    for z in (0.28, 0.58):
        for x in (-0.27, 0.27):
            box(cabinet, f"rack_rail_{z}_{x}", (0.035, 0.54, 0.035), (x, 0, z), m["dark"])
    door = model.part("door")
    box(door, "outer_skin", (0.64, 0.06, 0.82), (0, -0.03, 0.41), m["silver"])
    box(door, "inner_liner", (0.56, 0.025, 0.7), (0, 0.02, 0.4), m["white"])
    box(door, "control_panel", (0.58, 0.04, 0.1), (0, -0.065, 0.74), m["black"])
    box(door, "handle", (0.44, 0.07, 0.04), (0, -0.11, 0.62), m["dark"])
    for index, x in enumerate((-0.17, -0.09, -0.01, 0.07, 0.15)):
        cyl(
            door,
            f"button_{index}",
            0.014,
            0.02,
            (x, -0.09, 0.75),
            m["orange"] if index == 4 else m["steel"],
            (PI / 2, 0, 0),
        )
    revolute(model, "door_open", cabinet, door, (0, -0.31, 0.08), (1, 0, 0), 0, 1.65, 180)
    detergent = model.part("detergent_door")
    box(detergent, "cup", (0.18, 0.035, 0.13), (0, 0, 0.065), m["white"])
    box(detergent, "latch", (0.06, 0.04, 0.025), (0, -0.03, 0.11), m["dark"])
    revolute(model, "detergent_open", door, detergent, (0.13, 0.04, 0.33), (1, 0, 0), 0, 1.4, 10)
    for label, z in (("lower_rack", 0.27), ("upper_rack", 0.57)):
        rack = model.part(label)
        box(rack, "floor", (0.52, 0.48, 0.025), (0, 0.24, 0.015), m["steel"])
        for x in (-0.25, 0.25):
            box(rack, f"side_{x}", (0.025, 0.5, 0.16), (x, 0.24, 0.08), m["steel"])
        for index in range(8):
            x = -0.21 + index * 0.06
            box(rack, f"tine_{index}", (0.012, 0.4, 0.18), (x, 0.24, 0.1), m["steel"])
        for x in (-0.24, 0.24):
            cyl(rack, f"roller_{x}", 0.035, 0.035, (x, 0.18, 0), m["dark"], (0, PI / 2, 0))
        prismatic(model, f"{label}_slide", cabinet, rack, (0, -0.26, z), (0, -1, 0), 0, 0.46, 80)
    lower_arm = model.part("lower_spray_arm")
    cyl(lower_arm, "hub", 0.035, 0.04, (0, 0, 0), m["dark"])
    for angle in (0, 2 * PI / 3, 4 * PI / 3):
        box(
            lower_arm,
            f"blade_{angle}",
            (0.42, 0.035, 0.025),
            (math.cos(angle) * 0.16, math.sin(angle) * 0.16, 0),
            m["steel"],
            (0, 0, angle),
        )
    continuous(model, "lower_arm_spin", cabinet, lower_arm, (0, 0, 0.12), (0, 0, 1), 20)
    upper_arm = model.part("upper_spray_arm")
    cyl(upper_arm, "hub", 0.03, 0.035, (0, 0, 0), m["dark"])
    for angle in (0, PI):
        box(
            upper_arm,
            f"blade_{angle}",
            (0.36, 0.03, 0.02),
            (math.cos(angle) * 0.14, 0, 0),
            m["steel"],
            (0, 0, angle),
        )
    continuous(model, "upper_arm_spin", cabinet, upper_arm, (0, 0.2, 0.48), (0, 0, 1), 20)
    return model


def build_communications_satellite() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_communications_satellite")
    m = materials(model)
    bus = model.part("equipment_bus")
    box(bus, "bus_shell", (0.62, 0.52, 0.72), (0, 0, 0), m["white"])
    for face in (-1, 1):
        box(bus, f"radiator_{face}", (0.48, 0.025, 0.48), (0, face * 0.272, 0), m["silver"])
    for x in (-0.22, 0.22):
        cyl(bus, f"optical_sensor_{x}", 0.045, 0.04, (x, -0.29, 0.13), m["blue"], (PI / 2, 0, 0))
    for z in (-0.25, 0.25):
        box(bus, f"deck_{z}", (0.68, 0.58, 0.05), (0, 0, z), m["steel"])
    for side in (-1, 1):
        inner = model.part(f"solar_inner_{side}")
        box(inner, "panel", (0.72, 0.06, 0.58), (side * 0.36, 0, 0), m["blue"])
        for index in range(5):
            box(
                inner,
                f"cell_line_{index}",
                (0.012, 0.065, 0.54),
                (side * (0.08 + index * 0.14), -0.04, 0),
                m["silver"],
            )
        cyl(inner, "hinge_barrel", 0.045, 0.62, (0, 0, 0), m["dark"])
        revolute(
            model,
            f"array_root_fold_{side}",
            bus,
            inner,
            (side * 0.34, 0, 0),
            (0, 0, 1),
            -1.55,
            1.55,
            100,
        )
        outer = model.part(f"solar_outer_{side}")
        box(outer, "panel", (0.72, 0.06, 0.58), (side * 0.36, 0, 0), m["blue"])
        for index in range(5):
            box(
                outer,
                f"cell_line_{index}",
                (0.012, 0.065, 0.54),
                (side * (0.08 + index * 0.14), -0.04, 0),
                m["silver"],
            )
        revolute(
            model,
            f"array_tip_fold_{side}",
            inner,
            outer,
            (side * 0.72, 0, 0),
            (0, 0, 1),
            -1.55,
            1.55,
            80,
        )
    turntable = model.part("dish_turntable")
    cyl(turntable, "base_ring", 0.14, 0.11, (0, 0, 0.055), m["dark"])
    cyl(turntable, "bearing", 0.1, 0.14, (0, 0, 0.15), m["steel"])
    continuous(model, "dish_pan", bus, turntable, (0, 0, 0.39), (0, 0, 1), 120)
    dish = model.part("dish")
    cyl(dish, "reflector", 0.34, 0.045, (0, 0, 0.06), m["silver"])
    cyl(dish, "feed_boom", 0.025, 0.38, (0, 0, 0.23), m["dark"])
    cyl(dish, "feed_horn", 0.07, 0.1, (0, 0, 0.46), m["white"])
    for angle in (0, 2 * PI / 3, 4 * PI / 3):
        box(
            dish,
            f"feed_strut_{angle}",
            (0.03, 0.03, 0.38),
            (math.cos(angle) * 0.14, math.sin(angle) * 0.14, 0.22),
            m["dark"],
            (0, 0.38, angle),
        )
    revolute(model, "dish_tilt", turntable, dish, (0, 0, 0.22), (0, 1, 0), -0.25, 1.35, 100)
    cover = model.part("instrument_cover")
    box(cover, "panel", (0.26, 0.04, 0.24), (0.13, 0, 0), m["white"])
    box(cover, "latch", (0.07, 0.05, 0.04), (0.23, -0.03, 0), m["dark"])
    revolute(model, "cover_open", bus, cover, (-0.13, -0.29, -0.1), (0, -1, 0), 0, 1.6, 20)
    return model


def build_benchtop_cnc_mill() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_benchtop_cnc_mill")
    m = materials(model)
    enclosure = model.part("enclosure")
    box(enclosure, "base", (1.15, 0.88, 0.12), (0, 0, 0.06), m["dark"])
    box(enclosure, "rear", (1.08, 0.08, 1.05), (0, 0.4, 0.58), m["dark"])
    for x in (-0.54, 0.54):
        box(enclosure, f"side_{x}", (0.08, 0.82, 1.05), (x, 0, 0.58), m["dark"])
    box(enclosure, "roof", (1.16, 0.9, 0.09), (0, 0, 1.1), m["dark"])
    box(enclosure, "front_sill", (1.05, 0.08, 0.13), (0, -0.4, 0.17), m["dark"])
    for x in (-0.49, 0.49):
        box(enclosure, f"door_jamb_{x}", (0.07, 0.08, 0.86), (x, -0.4, 0.62), m["dark"])
    box(enclosure, "control_panel", (0.05, 0.32, 0.38), (0.585, -0.2, 0.72), m["black"])
    box(enclosure, "display", (0.02, 0.2, 0.12), (0.62, -0.2, 0.79), m["blue"])
    cyl(enclosure, "emergency_stop", 0.055, 0.055, (0.62, -0.2, 0.64), m["red"], (0, PI / 2, 0))
    x_stage = model.part("x_stage")
    box(x_stage, "carriage", (0.72, 0.48, 0.1), (0, 0, 0.05), m["steel"])
    for y in (-0.18, 0.18):
        cyl(x_stage, f"linear_rail_{y}", 0.025, 0.76, (0, y, -0.02), m["silver"], (0, PI / 2, 0))
    prismatic(model, "table_x", enclosure, x_stage, (0, 0, 0.25), (1, 0, 0), -0.2, 0.2, 500)
    y_stage = model.part("y_stage")
    box(y_stage, "saddle", (0.58, 0.42, 0.09), (0, 0, 0.045), m["dark"])
    box(y_stage, "bellows", (0.54, 0.38, 0.06), (0, 0.17, 0.1), m["black"])
    prismatic(model, "table_y", x_stage, y_stage, (0, 0, 0.1), (0, 1, 0), -0.14, 0.14, 500)
    table = model.part("worktable")
    box(table, "table", (0.62, 0.38, 0.08), (0, 0, 0.04), m["steel"])
    for index in range(6):
        box(
            table,
            f"t_slot_{index}",
            (0.025, 0.36, 0.012),
            (-0.25 + index * 0.1, 0, 0.085),
            m["black"],
        )
    fixed(model, "saddle_to_table", y_stage, table, (0, 0, 0.1))
    head = model.part("spindle_head")
    box(head, "carriage", (0.32, 0.3, 0.42), (0, 0, 0.12), m["white"])
    box(head, "motor", (0.28, 0.28, 0.32), (0, 0, 0.43), m["dark"])
    cyl(head, "quill", 0.075, 0.3, (0, 0, -0.12), m["steel"])
    prismatic(model, "spindle_z", enclosure, head, (0, 0.18, 0.78), (0, 0, 1), -0.28, 0.12, 500)
    spindle = model.part("spindle")
    cyl(spindle, "nose", 0.055, 0.18, (0, 0, -0.09), m["silver"])
    cyl(spindle, "tool", 0.018, 0.2, (0, 0, -0.26), m["steel"])
    continuous(model, "spindle_spin", head, spindle, (0, 0, -0.25), (0, 0, 1), 120)
    door = model.part("safety_door")
    box(door, "frame", (0.52, 0.05, 0.76), (0.26, 0, 0.38), m["steel"])
    box(door, "window", (0.43, 0.018, 0.66), (0.26, -0.035, 0.38), m["glass"])
    box(door, "handle", (0.04, 0.08, 0.24), (0.48, -0.06, 0.4), m["orange"])
    prismatic(model, "door_slide", enclosure, door, (-0.48, -0.42, 0.23), (1, 0, 0), 0, 0.46, 100)
    return model


def build_wall_bed() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_wall_bed")
    m = materials(model)
    cabinet = model.part("cabinet")
    box(cabinet, "back", (1.95, 0.08, 2.5), (0, 0.25, 1.25), m["wood"])
    for x in (-0.96, 0.96):
        box(cabinet, f"side_{x}", (0.08, 0.5, 2.5), (x, 0, 1.25), m["wood"])
    box(cabinet, "top", (2.0, 0.5, 0.1), (0, 0, 2.45), m["wood"])
    box(cabinet, "headboard", (1.78, 0.12, 0.65), (0, 0.12, 0.48), m["fabric"])
    cyl(cabinet, "bed_hinge", 0.07, 1.78, (0, -0.08, 0.45), m["steel"], (0, PI / 2, 0))
    bed = model.part("bed_frame")
    box(bed, "deck", (1.75, 0.12, 2.05), (0, 0, 1.02), m["dark"])
    box(bed, "mattress", (1.62, 0.22, 1.9), (0, -0.15, 1.05), m["cream"])
    for x in (-0.84, 0.84):
        box(bed, f"side_rail_{x}", (0.08, 0.24, 2.05), (x, 0, 1.02), m["wood"])
    for z in (0.08, 1.98):
        box(bed, f"cross_rail_{z}", (1.74, 0.24, 0.08), (0, 0, z), m["wood"])
    revolute(model, "bed_fold", cabinet, bed, (0, -0.12, 0.45), (1, 0, 0), 0, 1.55, 900)
    for index, x in enumerate((-0.68, 0.68)):
        leg = model.part(f"support_leg_{index}")
        box(leg, "leg", (0.08, 0.1, 0.72), (0, 0, -0.34), m["steel"])
        box(leg, "foot", (0.18, 0.18, 0.06), (0, 0, -0.72), m["rubber"])
        cyl(leg, "pivot", 0.045, 0.16, (0, 0, 0), m["dark"], (0, PI / 2, 0))
        revolute(model, f"leg_unfold_{index}", bed, leg, (x, 0, 1.88), (0, 1, 0), 0, 1.5, 150)
    for index, x in enumerate((-0.49, 0.49)):
        door = model.part(f"cabinet_door_{index}")
        inward = 0.46 if x < 0 else -0.46
        box(door, "slab", (0.92, 0.07, 2.25), (inward, 0, 0), m["wood"])
        box(door, "recess", (0.72, 0.03, 1.8), (inward, -0.05, 0), m["dark"])
        handle_x = inward + (0.32 if x < 0 else -0.32)
        cyl(door, "handle", 0.025, 0.38, (handle_x, -0.09, 0.03), m["steel"])
        axis = (0, 0, 1)
        revolute(
            model,
            f"door_open_{index}",
            cabinet,
            door,
            ((-0.94 if x < 0 else 0.94), -0.25, 1.28),
            axis,
            -1.7 if x < 0 else 0,
            0 if x < 0 else 1.7,
            80,
        )
    return model


def build_self_propelled_crop_sprayer() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_crop_sprayer")
    m = materials(model)
    chassis = model.part("chassis")
    box(chassis, "main_frame", (1.5, 2.35, 0.24), (0, 0, 0.72), m["green"])
    box(chassis, "front_deck", (1.42, 0.55, 0.12), (0, 0.85, 0.9), m["green"])
    box(chassis, "rear_deck", (1.42, 0.5, 0.12), (0, -0.92, 0.9), m["green"])
    box(chassis, "cab", (1.2, 0.86, 1.15), (0, 0.58, 1.48), m["green"])
    for x in (-0.58, 0.58):
        box(chassis, f"cab_window_{x}", (0.025, 0.64, 0.72), (x, 0.58, 1.55), m["glass"])
    box(chassis, "windshield", (1.05, 0.025, 0.72), (0, 0.99, 1.55), m["glass"])
    cyl(chassis, "tank", 0.58, 1.28, (0, -0.55, 1.32), m["cream"], (PI / 2, 0, 0))
    for index, x in enumerate((-0.5, 0.5)):
        knuckle = model.part(f"steering_knuckle_{index}")
        box(knuckle, "upright", (0.16, 0.18, 0.42), (0, 0, 0), m["dark"])
        revolute(
            model, f"steer_{index}", chassis, knuckle, (x, 0.86, 0.55), (0, 0, 1), -0.55, 0.55, 300
        )
        front_wheel = model.part(f"front_wheel_{index}")
        wheel(front_wheel, m["rubber"], m["yellow"], 0.42, 0.22)
        continuous(
            model,
            f"front_wheel_spin_{index}",
            knuckle,
            front_wheel,
            (0, 0.18, -0.12),
            (0, 1, 0),
            120,
        )
    for index, (x, y) in enumerate(((-0.5, -0.88), (0.5, -0.88))):
        rear_wheel = model.part(f"rear_wheel_{index}")
        wheel(rear_wheel, m["rubber"], m["yellow"], 0.46, 0.24)
        continuous(
            model, f"rear_wheel_spin_{index}", chassis, rear_wheel, (x, y, 0.46), (0, 1, 0), 120
        )
    mast = model.part("boom_mast")
    box(mast, "mast", (0.24, 0.25, 1.1), (0, 0, 0.5), m["dark"])
    cyl(mast, "lift_ram", 0.05, 0.9, (0.15, 0, 0.45), m["silver"])
    prismatic(model, "boom_height", chassis, mast, (0, -1.15, 0.9), (0, 0, 1), 0, 0.65, 900)
    center = model.part("center_boom")
    box(center, "bar", (1.2, 0.14, 0.12), (0, 0, 0), m["yellow"])
    fixed(model, "mast_to_center_boom", mast, center, (0, 0, 1.0))
    for side in (-1, 1):
        inner = model.part(f"boom_inner_{side}")
        box(inner, "beam", (1.2, 0.13, 0.11), (side * 0.58, 0, 0), m["yellow"])
        box(inner, "hose", (1.12, 0.035, 0.035), (side * 0.58, -0.08, 0), m["black"])
        for index in range(4):
            x = side * (0.2 + index * 0.28)
            cyl(inner, f"nozzle_stem_{index}", 0.012, 0.24, (x, 0, -0.16), m["dark"])
            sphere(inner, f"nozzle_{index}", 0.03, (x, 0, -0.3), m["orange"])
        revolute(
            model,
            f"boom_inner_fold_{side}",
            center,
            inner,
            (side * 0.6, 0, 0),
            (0, 0, 1),
            -2.4 if side < 0 else 0,
            0 if side < 0 else 2.4,
            500,
        )
        outer = model.part(f"boom_outer_{side}")
        box(outer, "beam", (1.35, 0.12, 0.1), (side * 0.65, 0, 0), m["yellow"])
        box(outer, "hose", (1.28, 0.035, 0.035), (side * 0.65, -0.08, 0), m["black"])
        for index in range(5):
            x = side * (0.15 + index * 0.28)
            cyl(outer, f"nozzle_stem_{index}", 0.012, 0.24, (x, 0, -0.16), m["dark"])
            sphere(outer, f"nozzle_{index}", 0.03, (x, 0, -0.3), m["orange"])
        revolute(
            model,
            f"boom_outer_fold_{side}",
            inner,
            outer,
            (side * 1.16, 0, 0),
            (0, 0, 1),
            -2.6 if side < 0 else 0,
            0 if side < 0 else 2.6,
            400,
        )
    return model


def build_video_tripod() -> ArticulatedObject:
    model = ArticulatedObject(name="handcrafted_video_tripod")
    m = materials(model)
    hub = model.part("tripod_hub")
    cyl(hub, "hub", 0.13, 0.18, (0, 0, 1.02), m["dark"])
    cyl(hub, "bowl", 0.1, 0.08, (0, 0, 1.15), m["steel"])
    sphere(hub, "bubble_level", 0.025, (0.07, 0, 1.2), m["green"])
    for index, angle in enumerate((0, 2 * PI / 3, 4 * PI / 3)):
        mount = model.part(f"leg_mount_{index}")
        box(mount, "hinge_block", (0.14, 0.12, 0.18), (0.06, 0, -0.08), m["dark"])
        box(mount, "upper_tube", (0.12, 0.1, 0.72), (0.26, 0, -0.38), m["dark"], (0, -0.55, 0))
        revolute(
            model,
            f"leg_spread_{index}",
            hub,
            mount,
            (0, 0, 1.02),
            (math.sin(angle), -math.cos(angle), 0),
            0.3,
            0.95,
            200,
        )
        mid = model.part(f"leg_mid_{index}")
        box(mid, "mid_tube", (0.09, 0.075, 0.72), (0.26, 0, -0.38), m["steel"], (0, -0.55, 0))
        box(mid, "clamp", (0.15, 0.13, 0.1), (0.05, 0, -0.08), m["black"])
        prismatic(
            model,
            f"leg_mid_extend_{index}",
            mount,
            mid,
            (0.35, 0, -0.6),
            (0.48, 0, -0.88),
            0,
            0.36,
            120,
        )
        lower = model.part(f"leg_lower_{index}")
        box(lower, "lower_tube", (0.07, 0.06, 0.68), (0.24, 0, -0.35), m["dark"], (0, -0.55, 0))
        box(lower, "foot", (0.16, 0.14, 0.08), (0.48, 0, -0.66), m["rubber"], (0, -0.55, 0))
        prismatic(
            model,
            f"leg_lower_extend_{index}",
            mid,
            lower,
            (0.36, 0, -0.62),
            (0.48, 0, -0.88),
            0,
            0.32,
            100,
        )
    column = model.part("center_column")
    cyl(column, "column", 0.035, 0.78, (0, 0, 0.32), m["steel"])
    cyl(column, "collar", 0.055, 0.08, (0, 0, 0), m["dark"])
    prismatic(model, "column_height", hub, column, (0, 0, 1.16), (0, 0, 1), 0, 0.42, 180)
    pan = model.part("pan_head")
    cyl(pan, "fluid_base", 0.11, 0.12, (0, 0, 0.06), m["dark"])
    box(pan, "yoke_base", (0.2, 0.16, 0.08), (0, 0, 0.15), m["steel"])
    continuous(model, "head_pan", column, pan, (0, 0, 0.72), (0, 0, 1), 80)
    tilt = model.part("tilt_head")
    for x in (-0.09, 0.09):
        box(tilt, f"yoke_cheek_{x}", (0.05, 0.16, 0.22), (x, 0, 0.08), m["dark"])
    cyl(tilt, "tilt_axle", 0.03, 0.24, (0, 0, 0.1), m["steel"], (0, PI / 2, 0))
    box(tilt, "camera_plate", (0.18, 0.34, 0.05), (0, 0.04, 0.23), m["dark"])
    box(tilt, "rubber_pad", (0.14, 0.28, 0.025), (0, 0.04, 0.27), m["rubber"])
    cyl(tilt, "mount_screw", 0.015, 0.04, (0, 0.04, 0.3), m["silver"])
    box(tilt, "pan_handle", (0.05, 0.55, 0.05), (0.14, -0.24, 0.08), m["black"], (0.2, 0, -0.25))
    revolute(model, "head_tilt", pan, tilt, (0, 0, 0.2), (1, 0, 0), -0.9, 0.9, 100)
    return model


BUILDERS = {
    "compact_excavator": build_compact_excavator,
    "powered_hospital_bed": build_powered_hospital_bed,
    "folding_bicycle": build_folding_bicycle,
    "sliding_compound_miter_saw": build_sliding_compound_miter_saw,
    "dishwasher": build_dishwasher,
    "communications_satellite": build_communications_satellite,
    "benchtop_cnc_mill": build_benchtop_cnc_mill,
    "wall_bed": build_wall_bed,
    "self_propelled_crop_sprayer": build_self_propelled_crop_sprayer,
    "video_tripod": build_video_tripod,
}

REQUIRED = {
    "compact_excavator": (("boom", "stick", "bucket"), ("body_swing", "boom_raise", "bucket_curl")),
    "powered_hospital_bed": (
        ("lift_column", "backrest", "leg_section"),
        ("bed_height", "backrest_raise", "leg_raise"),
    ),
    "folding_bicycle": (
        ("rear_wheel", "front_wheel", "crank"),
        ("frame_fold", "steering", "crank_spin"),
    ),
    "sliding_compound_miter_saw": (
        ("rotary_table", "saw_arm", "blade"),
        ("miter_angle", "head_slide", "saw_plunge"),
    ),
    "dishwasher": (
        ("door", "lower_rack", "upper_rack"),
        ("door_open", "lower_rack_slide", "lower_arm_spin"),
    ),
    "communications_satellite": (
        ("equipment_bus", "dish", "instrument_cover"),
        ("dish_pan", "dish_tilt", "cover_open"),
    ),
    "benchtop_cnc_mill": (
        ("x_stage", "y_stage", "spindle_head"),
        ("table_x", "table_y", "spindle_z"),
    ),
    "wall_bed": (
        ("bed_frame", "support_leg_0", "cabinet_door_0"),
        ("bed_fold", "leg_unfold_0", "door_open_0"),
    ),
    "self_propelled_crop_sprayer": (
        ("boom_mast", "boom_inner_1", "front_wheel_0"),
        ("boom_height", "boom_inner_fold_1", "steer_0"),
    ),
    "video_tripod": (
        ("leg_mount_0", "leg_mid_0", "tilt_head"),
        ("leg_spread_0", "leg_mid_extend_0", "head_tilt"),
    ),
}


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    required_parts, required_joints = REQUIRED[TARGET_OBJECT]
    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    for name in required_parts:
        ctx.check(f"required part: {name}", name in part_names)
    for name in required_joints:
        ctx.check(f"required articulation: {name}", name in joint_names)
    ctx.check("detailed visual count", sum(len(part.visuals) for part in object_model.parts) >= 24)
    ctx.check("multiple useful articulations", len(object_model.articulations) >= 5)
    return ctx.report()


object_model = BUILDERS[TARGET_OBJECT]()
