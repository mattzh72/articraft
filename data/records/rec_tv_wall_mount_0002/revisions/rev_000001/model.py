from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

ARM1_LENGTH = 0.215
ARM2_LENGTH = 0.205
TILT_PIVOT_X = 0.060


def box_at(dx: float, dy: float, dz: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(dx, dy, dz).translate(center)


def cylinder_x(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length / 2.0, center[1], center[2]))
    )


def cylinder_y(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] - length / 2.0, center[2]))
    )


def cylinder_z(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - length / 2.0))
    )


def slot_cut_yz(
    x_center: float,
    y: float,
    z: float,
    length: float,
    diameter: float,
    *,
    angle: float = 0.0,
    x_span: float = 0.120,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .slot2D(length, diameter, angle)
        .extrude(x_span)
        .translate((x_center - x_span / 2.0, 0.0, 0.0))
    )


def slot_cut_xz(
    x: float,
    y_center: float,
    z: float,
    length: float,
    diameter: float,
    *,
    angle: float = 0.0,
    y_span: float = 0.100,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .slot2D(length, diameter, angle)
        .extrude(y_span)
        .translate((0.0, y_center - y_span / 2.0, 0.0))
    )


def extrude_y_profile(
    points: list[tuple[float, float]],
    *,
    y_center: float,
    thickness: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, y_center - thickness / 2.0, 0.0))
    )


def extrude_x_profile(
    points: list[tuple[float, float]],
    *,
    x_center: float,
    thickness: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((x_center - thickness / 2.0, 0.0, 0.0))
    )


def fuse_all(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def cut_all(base: cq.Workplane, *cutters: cq.Workplane) -> cq.Workplane:
    result = base
    for cutter in cutters:
        result = result.cut(cutter)
    return result


def add_mesh_visual(part, shape: cq.Workplane, *, filename: str, material, name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, filename, assets=ASSETS),
        material=material,
        name=name,
    )


def make_wall_plate_components() -> dict[str, cq.Workplane]:
    mount_cutters = [
        slot_cut_yz(-0.053, y, z, 0.064, 0.012, angle=90.0, x_span=0.018)
        for y in (-0.196, 0.196)
        for z in (-0.082, 0.082)
    ]
    cable_passage = slot_cut_yz(-0.040, 0.0, -0.062, 0.084, 0.018, angle=0.0, x_span=0.032)
    service_window = slot_cut_yz(-0.030, 0.0, 0.054, 0.118, 0.024, angle=90.0, x_span=0.028)

    back_plate = box_at(0.010, 0.560, 0.260, (-0.053, 0.0, 0.0))
    back_plate = cut_all(back_plate, *mount_cutters, cable_passage)

    spine_block = fuse_all(
        box_at(0.034, 0.176, 0.182, (-0.031, 0.0, 0.0)),
        box_at(0.020, 0.070, 0.092, (-0.014, 0.0, 0.0)),
        extrude_y_profile(
            [(-0.044, -0.082), (-0.018, -0.024), (-0.018, 0.024), (-0.044, 0.070)],
            y_center=0.022,
            thickness=0.010,
        ),
        extrude_y_profile(
            [(-0.044, -0.070), (-0.018, -0.024), (-0.018, 0.024), (-0.044, 0.082)],
            y_center=-0.022,
            thickness=0.010,
        ),
    )
    spine_block = cut_all(spine_block, cable_passage, service_window)

    shoulder_bracket = fuse_all(
        box_at(0.016, 0.012, 0.104, (-0.008, 0.027, 0.0)),
        box_at(0.016, 0.012, 0.104, (-0.008, -0.027, 0.0)),
        box_at(0.010, 0.038, 0.074, (-0.018, 0.0, 0.0)),
        box_at(0.006, 0.020, 0.112, (-0.003, 0.0, 0.0)),
        box_at(0.006, 0.056, 0.016, (-0.018, 0.0, 0.046)),
        box_at(0.006, 0.056, 0.016, (-0.018, 0.0, -0.046)),
    )

    cover_set = fuse_all(
        box_at(0.004, 0.126, 0.080, (-0.046, 0.0, 0.070)),
        box_at(0.004, 0.126, 0.080, (-0.046, 0.0, -0.070)),
    )

    cover_hardware = fuse_all(
        cylinder_x(0.003, 0.0042, (-0.0445, 0.046, 0.100)),
        cylinder_x(0.003, 0.0042, (-0.0445, -0.046, 0.100)),
        cylinder_x(0.003, 0.0042, (-0.0445, 0.046, 0.040)),
        cylinder_x(0.003, 0.0042, (-0.0445, -0.046, 0.040)),
        cylinder_x(0.003, 0.0042, (-0.0445, 0.046, -0.040)),
        cylinder_x(0.003, 0.0042, (-0.0445, -0.046, -0.040)),
        cylinder_x(0.003, 0.0042, (-0.0445, 0.046, -0.100)),
        cylinder_x(0.003, 0.0042, (-0.0445, -0.046, -0.100)),
        cylinder_z(0.008, 0.008, (-0.008, 0.0, 0.050)),
        cylinder_z(0.008, 0.008, (-0.008, 0.0, -0.050)),
    )

    return {
        "back_plate": back_plate,
        "spine_block": spine_block,
        "shoulder_bracket": shoulder_bracket,
        "cover_set": cover_set,
        "cover_hardware": cover_hardware,
    }


def make_primary_arm_components() -> dict[str, cq.Workplane]:
    cable_slot = slot_cut_xz(0.108, 0.0, 0.0, 0.088, 0.018, angle=0.0, y_span=0.034)
    spring_hole_a = cylinder_y(0.034, 0.006, (0.074, 0.0, 0.0))
    spring_hole_b = cylinder_y(0.034, 0.006, (0.154, 0.0, 0.0))

    side_webs = fuse_all(
        extrude_y_profile(
            [
                (0.012, -0.032),
                (0.048, -0.032),
                (0.094, -0.026),
                (0.160, -0.024),
                (0.206, -0.030),
                (0.206, 0.030),
                (0.160, 0.024),
                (0.094, 0.026),
                (0.048, 0.032),
                (0.012, 0.032),
            ],
            y_center=0.022,
            thickness=0.006,
        ),
        extrude_y_profile(
            [
                (0.012, -0.032),
                (0.048, -0.032),
                (0.094, -0.026),
                (0.160, -0.024),
                (0.206, -0.030),
                (0.206, 0.030),
                (0.160, 0.024),
                (0.094, 0.026),
                (0.048, 0.032),
                (0.012, 0.032),
            ],
            y_center=-0.022,
            thickness=0.006,
        ),
    )
    side_webs = cut_all(side_webs, cable_slot, spring_hole_a, spring_hole_b)

    shoulder_block = box_at(0.018, 0.032, 0.056, (0.009, 0.0, 0.0))

    bridge_pack = cut_all(
        fuse_all(
            box_at(0.026, 0.034, 0.014, (0.040, 0.0, -0.019)),
            box_at(0.090, 0.038, 0.008, (0.108, 0.0, 0.020)),
            box_at(0.082, 0.032, 0.006, (0.110, 0.0, -0.021)),
            box_at(0.080, 0.028, 0.003, (0.108, 0.0, -0.0185)),
        ),
        cable_slot,
    )

    spring_housing = fuse_all(
        cylinder_x(0.110, 0.012, (0.114, 0.0, 0.028)),
        box_at(0.014, 0.026, 0.018, (0.074, 0.0, 0.028)),
        box_at(0.014, 0.026, 0.018, (0.154, 0.0, 0.028)),
    )

    elbow_clevis = fuse_all(
        box_at(0.018, 0.010, 0.074, (0.205, 0.019, 0.0)),
        box_at(0.018, 0.010, 0.074, (0.205, -0.019, 0.0)),
        box_at(0.012, 0.034, 0.016, (0.190, 0.0, -0.015)),
    )

    pivot_hardware = fuse_all(
        cylinder_z(0.008, 0.008, (0.009, 0.0, 0.032)),
        cylinder_z(0.008, 0.008, (0.009, 0.0, -0.032)),
        cylinder_z(0.008, 0.008, (0.205, 0.0, 0.040)),
        cylinder_z(0.008, 0.008, (0.205, 0.0, -0.040)),
    )

    return {
        "side_webs": side_webs,
        "shoulder_block": shoulder_block,
        "bridge_pack": bridge_pack,
        "spring_housing": spring_housing,
        "elbow_clevis": elbow_clevis,
        "pivot_hardware": pivot_hardware,
    }


def make_secondary_arm_components() -> dict[str, cq.Workplane]:
    cable_slot = slot_cut_xz(0.106, 0.0, 0.0, 0.084, 0.018, angle=0.0, y_span=0.036)
    spring_hole = cylinder_y(0.036, 0.0062, (0.126, 0.0, 0.0))

    side_webs = fuse_all(
        extrude_y_profile(
            [
                (0.012, -0.031),
                (0.046, -0.031),
                (0.092, -0.025),
                (0.148, -0.024),
                (0.194, -0.030),
                (0.194, 0.030),
                (0.148, 0.024),
                (0.092, 0.025),
                (0.046, 0.031),
                (0.012, 0.031),
            ],
            y_center=0.023,
            thickness=0.006,
        ),
        extrude_y_profile(
            [
                (0.012, -0.031),
                (0.046, -0.031),
                (0.092, -0.025),
                (0.148, -0.024),
                (0.194, -0.030),
                (0.194, 0.030),
                (0.148, 0.024),
                (0.092, 0.025),
                (0.046, 0.031),
                (0.012, 0.031),
            ],
            y_center=-0.023,
            thickness=0.006,
        ),
    )
    side_webs = cut_all(side_webs, cable_slot, spring_hole)

    shoulder_block = box_at(0.018, 0.034, 0.058, (0.009, 0.0, 0.0))

    bridge_pack = cut_all(
        fuse_all(
            box_at(0.028, 0.036, 0.014, (0.042, 0.0, 0.019)),
            box_at(0.082, 0.040, 0.008, (0.102, 0.0, -0.020)),
            box_at(0.080, 0.032, 0.006, (0.108, 0.0, 0.020)),
            box_at(0.018, 0.042, 0.018, (0.182, 0.0, 0.0)),
        ),
        cable_slot,
    )

    spring_housing = fuse_all(
        cylinder_x(0.102, 0.011, (0.112, 0.0, -0.028)),
        box_at(0.014, 0.028, 0.018, (0.074, 0.0, -0.028)),
        box_at(0.014, 0.028, 0.018, (0.150, 0.0, -0.028)),
    )

    pan_flange = fuse_all(
        box_at(0.020, 0.058, 0.018, (0.195, 0.0, -0.004)),
        cylinder_z(0.008, 0.008, (0.189, 0.0, -0.004)),
    )

    pivot_hardware = fuse_all(
        cylinder_z(0.008, 0.008, (0.009, 0.0, 0.033)),
        cylinder_z(0.008, 0.008, (0.009, 0.0, -0.033)),
        cylinder_z(0.008, 0.008, (0.189, 0.0, 0.030)),
        cylinder_z(0.008, 0.008, (0.189, 0.0, -0.038)),
    )

    return {
        "side_webs": side_webs,
        "shoulder_block": shoulder_block,
        "bridge_pack": bridge_pack,
        "spring_housing": spring_housing,
        "pan_flange": pan_flange,
        "pivot_hardware": pivot_hardware,
    }


def make_pan_head_components() -> dict[str, cq.Workplane]:
    pan_base = fuse_all(
        box_at(0.016, 0.040, 0.054, (0.008, 0.0, 0.0)),
        box_at(0.018, 0.028, 0.082, (0.026, 0.0, 0.0)),
        cylinder_z(0.058, 0.010, (0.011, 0.0, 0.0)),
    )
    pan_base = cut_all(pan_base, cylinder_z(0.090, 0.006, (0.011, 0.0, 0.0)))

    tilt_ears = cut_all(
        fuse_all(
            box_at(0.012, 0.010, 0.190, (0.054, 0.050, 0.0)),
            box_at(0.012, 0.010, 0.190, (0.054, -0.050, 0.0)),
            box_at(0.008, 0.110, 0.016, (0.040, 0.0, 0.090)),
            box_at(0.008, 0.110, 0.016, (0.040, 0.0, -0.090)),
            box_at(0.012, 0.030, 0.064, (0.034, 0.0, 0.0)),
        ),
        cylinder_y(0.018, 0.0085, (0.054, 0.050, 0.0)),
        cylinder_y(0.018, 0.0085, (0.054, -0.050, 0.0)),
    )

    tie_bars = fuse_all(
        box_at(0.010, 0.116, 0.016, (0.036, 0.0, 0.090)),
        box_at(0.010, 0.116, 0.016, (0.036, 0.0, -0.090)),
        box_at(0.012, 0.032, 0.070, (0.030, 0.0, 0.0)),
        cylinder_x(0.004, 0.004, (0.044, 0.018, 0.060)),
        cylinder_x(0.004, 0.004, (0.044, -0.018, 0.060)),
        cylinder_x(0.004, 0.004, (0.044, 0.018, -0.060)),
        cylinder_x(0.004, 0.004, (0.044, -0.018, -0.060)),
    )

    return {
        "pan_base": pan_base,
        "tilt_ears": tilt_ears,
        "tie_bars": tie_bars,
    }


def make_tilt_frame_components() -> dict[str, cq.Workplane]:
    rail_slots = []
    for rail_y in (-0.168, 0.168):
        rail_slots.extend(
            [
                slot_cut_xz(0.018, rail_y, 0.098, 0.064, 0.010, angle=90.0, y_span=0.036),
                slot_cut_xz(0.018, rail_y, 0.032, 0.056, 0.010, angle=90.0, y_span=0.036),
                slot_cut_xz(0.018, rail_y, -0.032, 0.056, 0.010, angle=90.0, y_span=0.036),
                slot_cut_xz(0.018, rail_y, -0.098, 0.064, 0.010, angle=90.0, y_span=0.036),
            ]
        )
    center_slot = slot_cut_xz(0.020, 0.0, 0.0, 0.088, 0.016, angle=0.0, y_span=0.112)
    lower_slot = slot_cut_xz(0.020, 0.0, -0.148, 0.072, 0.014, angle=0.0, y_span=0.104)

    tilt_trunnion = cut_all(
        fuse_all(
            box_at(0.014, 0.090, 0.074, (0.007, 0.0, 0.0)),
            box_at(0.010, 0.056, 0.056, (0.013, 0.0, 0.0)),
            box_at(0.006, 0.028, 0.020, (0.011, 0.0, 0.030)),
            box_at(0.006, 0.028, 0.020, (0.011, 0.0, -0.030)),
        ),
        cylinder_y(0.096, 0.0082, (0.008, 0.0, 0.0)),
    )

    rail_frame = fuse_all(
        box_at(0.010, 0.028, 0.340, (0.018, 0.168, 0.0)),
        box_at(0.010, 0.028, 0.340, (0.018, -0.168, 0.0)),
        box_at(0.010, 0.356, 0.022, (0.016, 0.0, 0.148)),
        box_at(0.010, 0.356, 0.022, (0.016, 0.0, -0.148)),
    )
    rail_frame = cut_all(rail_frame, *rail_slots)

    brace_pack = cut_all(
        fuse_all(
            box_at(0.014, 0.156, 0.014, (0.016, 0.0, 0.0)),
            box_at(0.008, 0.096, 0.012, (0.020, 0.0, -0.148)),
            box_at(0.012, 0.080, 0.032, (0.010, 0.0, 0.0)),
            extrude_x_profile(
                [(0.052, 0.012), (0.148, 0.118), (0.148, 0.102), (0.070, 0.002)],
                x_center=0.014,
                thickness=0.006,
            ),
            extrude_x_profile(
                [(-0.148, 0.118), (-0.052, 0.012), (-0.070, 0.002), (-0.148, 0.102)],
                x_center=0.014,
                thickness=0.006,
            ),
            extrude_x_profile(
                [(0.070, -0.002), (0.148, -0.102), (0.148, -0.118), (0.052, -0.012)],
                x_center=0.014,
                thickness=0.006,
            ),
            extrude_x_profile(
                [(-0.148, -0.102), (-0.070, -0.002), (-0.052, -0.012), (-0.148, -0.118)],
                x_center=0.014,
                thickness=0.006,
            ),
        ),
        center_slot,
        lower_slot,
    )

    return {
        "tilt_trunnion": tilt_trunnion,
        "rail_frame": rail_frame,
        "brace_pack": brace_pack,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tv_wall_mount_study", assets=ASSETS)

    powder_coat = model.material("powder_coat", rgba=(0.14, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.36, 1.0))
    zinc = model.material("zinc", rgba=(0.70, 0.72, 0.74, 1.0))

    wall_plate = model.part("wall_plate")
    for visual_name, shape, material_name in (
        ("back_plate", make_wall_plate_components()["back_plate"], graphite),
        ("spine_block", make_wall_plate_components()["spine_block"], graphite),
        ("shoulder_bracket", make_wall_plate_components()["shoulder_bracket"], dark_steel),
        ("cover_set", make_wall_plate_components()["cover_set"], dark_steel),
        ("cover_hardware", make_wall_plate_components()["cover_hardware"], zinc),
    ):
        add_mesh_visual(
            wall_plate,
            shape,
            filename=f"wall_plate_{visual_name}.obj",
            material=material_name,
            name=visual_name,
        )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.064, 0.560, 0.260)),
        mass=6.0,
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
    )

    primary_arm = model.part("primary_arm")
    primary_components = make_primary_arm_components()
    for visual_name, material_name in (
        ("side_webs", powder_coat),
        ("shoulder_block", graphite),
        ("bridge_pack", powder_coat),
        ("spring_housing", dark_steel),
        ("elbow_clevis", graphite),
        ("pivot_hardware", zinc),
    ):
        add_mesh_visual(
            primary_arm,
            primary_components[visual_name],
            filename=f"primary_arm_{visual_name}.obj",
            material=material_name,
            name=visual_name,
        )
    primary_arm.inertial = Inertial.from_geometry(
        Box((0.220, 0.060, 0.084)),
        mass=2.7,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
    )

    secondary_arm = model.part("secondary_arm")
    secondary_components = make_secondary_arm_components()
    for visual_name, material_name in (
        ("side_webs", powder_coat),
        ("shoulder_block", graphite),
        ("bridge_pack", powder_coat),
        ("spring_housing", dark_steel),
        ("pan_flange", graphite),
        ("pivot_hardware", zinc),
    ):
        add_mesh_visual(
            secondary_arm,
            secondary_components[visual_name],
            filename=f"secondary_arm_{visual_name}.obj",
            material=material_name,
            name=visual_name,
        )
    secondary_arm.inertial = Inertial.from_geometry(
        Box((0.210, 0.065, 0.084)),
        mass=2.9,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    pan_head = model.part("pan_head")
    pan_components = make_pan_head_components()
    for visual_name, material_name in (
        ("pan_base", dark_steel),
        ("tilt_ears", graphite),
        ("tie_bars", zinc),
    ):
        add_mesh_visual(
            pan_head,
            pan_components[visual_name],
            filename=f"pan_head_{visual_name}.obj",
            material=material_name,
            name=visual_name,
        )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.070, 0.120, 0.200)),
        mass=2.0,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
    )

    tilt_frame = model.part("tilt_frame")
    tilt_components = make_tilt_frame_components()
    for visual_name, material_name in (
        ("tilt_trunnion", dark_steel),
        ("rail_frame", graphite),
        ("brace_pack", powder_coat),
    ):
        add_mesh_visual(
            tilt_frame,
            tilt_components[visual_name],
            filename=f"tilt_frame_{visual_name}.obj",
            material=material_name,
            name=visual_name,
        )
    tilt_frame.inertial = Inertial.from_geometry(
        Box((0.040, 0.370, 0.340)),
        mass=3.6,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_fold",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=-1.25, upper=1.40),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(ARM1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=1.6, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "head_pan",
        ArticulationType.CONTINUOUS,
        parent=secondary_arm,
        child=pan_head,
        origin=Origin(xyz=(ARM2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.8),
    )
    model.articulation(
        "frame_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_frame,
        origin=Origin(xyz=(TILT_PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=-0.45, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    wall_plate = object_model.get_part("wall_plate")
    primary_arm = object_model.get_part("primary_arm")
    secondary_arm = object_model.get_part("secondary_arm")
    pan_head = object_model.get_part("pan_head")
    tilt_frame = object_model.get_part("tilt_frame")
    shoulder_fold = object_model.get_articulation("shoulder_fold")
    elbow_fold = object_model.get_articulation("elbow_fold")
    head_pan = object_model.get_articulation("head_pan")
    frame_tilt = object_model.get_articulation("frame_tilt")
    wall_back = wall_plate.get_visual("back_plate")
    shoulder_bracket = wall_plate.get_visual("shoulder_bracket")
    cover_set = wall_plate.get_visual("cover_set")
    primary_shoulder = primary_arm.get_visual("shoulder_block")
    primary_elbow = primary_arm.get_visual("elbow_clevis")
    secondary_shoulder = secondary_arm.get_visual("shoulder_block")
    secondary_pan = secondary_arm.get_visual("pan_flange")
    pan_base = pan_head.get_visual("pan_base")
    pan_ears = pan_head.get_visual("tilt_ears")
    tilt_trunnion = tilt_frame.get_visual("tilt_trunnion")
    rail_frame = tilt_frame.get_visual("rail_frame")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "part_inventory",
        len(object_model.parts) == 5,
        f"expected 5 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "articulation_inventory",
        len(object_model.articulations) == 4,
        f"expected 4 articulations, found {len(object_model.articulations)}",
    )
    ctx.check(
        "feature_visuals_present",
        all(
            visual is not None
            for visual in (
                wall_back,
                shoulder_bracket,
                cover_set,
                primary_shoulder,
                primary_elbow,
                secondary_shoulder,
                secondary_pan,
                pan_base,
                pan_ears,
                tilt_trunnion,
                rail_frame,
            )
        ),
        "expected named mechanical feature visuals for covers, brackets, flanges, ears, and trunnions",
    )

    ctx.expect_contact(
        primary_arm,
        wall_plate,
        contact_tol=0.0015,
        elem_a=primary_shoulder,
        elem_b=shoulder_bracket,
        name="shoulder_joint_contact_rest",
    )
    ctx.expect_overlap(
        primary_arm,
        wall_plate,
        axes="yz",
        min_overlap=0.030,
        elem_a=primary_shoulder,
        elem_b=shoulder_bracket,
        name="shoulder_joint_register_rest",
    )
    ctx.expect_contact(
        secondary_arm,
        primary_arm,
        contact_tol=0.0015,
        elem_a=secondary_shoulder,
        elem_b=primary_elbow,
        name="elbow_joint_contact_rest",
    )
    ctx.expect_overlap(
        secondary_arm,
        primary_arm,
        axes="yz",
        min_overlap=0.030,
        elem_a=secondary_shoulder,
        elem_b=primary_elbow,
        name="elbow_joint_register_rest",
    )
    ctx.expect_contact(
        pan_head,
        secondary_arm,
        contact_tol=0.0015,
        elem_a=pan_base,
        elem_b=secondary_pan,
        name="pan_stack_contact_rest",
    )
    ctx.expect_overlap(
        pan_head,
        secondary_arm,
        axes="yz",
        min_overlap=0.018,
        elem_a=pan_base,
        elem_b=secondary_pan,
        name="pan_stack_register_rest",
    )
    ctx.expect_contact(
        tilt_frame,
        pan_head,
        contact_tol=0.0015,
        elem_a=tilt_trunnion,
        elem_b=pan_ears,
        name="tilt_bearing_contact_rest",
    )
    ctx.expect_overlap(
        tilt_frame,
        pan_head,
        axes="z",
        min_overlap=0.074,
        elem_a=tilt_trunnion,
        elem_b=pan_ears,
        name="tilt_bearing_vertical_register_rest",
    )
    ctx.expect_origin_gap(
        tilt_frame,
        pan_head,
        axis="x",
        min_gap=0.058,
        max_gap=0.062,
        name="tilt_bearing_offset_rest",
    )

    ctx.expect_origin_gap(
        tilt_frame,
        wall_plate,
        axis="x",
        min_gap=0.44,
        max_gap=0.53,
        name="resting_reach_from_wall",
    )
    ctx.expect_origin_distance(
        tilt_frame,
        wall_plate,
        axes="yz",
        max_dist=0.02,
        name="rest_pose_centered_on_wall_plate",
    )
    ctx.expect_within(
        tilt_frame,
        wall_plate,
        axes="y",
        margin=0.0,
        inner_elem=rail_frame,
        outer_elem=wall_back,
        name="tilt_frame_within_wall_plate_width_rest",
    )
    ctx.expect_overlap(
        tilt_frame,
        wall_plate,
        axes="z",
        min_overlap=0.24,
        elem_a=rail_frame,
        elem_b=wall_back,
        name="frame_stays_within_wall_plate_vertical_band",
    )

    with ctx.pose({shoulder_fold: 0.65, elbow_fold: -1.85}):
        ctx.expect_contact(
            primary_arm,
            wall_plate,
            contact_tol=0.0015,
            elem_a=primary_shoulder,
            elem_b=shoulder_bracket,
            name="shoulder_joint_contact_folded",
        )
        ctx.expect_contact(
            secondary_arm,
            primary_arm,
            contact_tol=0.0015,
            elem_a=secondary_shoulder,
            elem_b=primary_elbow,
            name="elbow_joint_contact_folded",
        )
        ctx.expect_contact(
            pan_head,
            secondary_arm,
            contact_tol=0.0015,
            elem_a=pan_base,
            elem_b=secondary_pan,
            name="pan_stack_contact_folded",
        )
        ctx.expect_contact(
            tilt_frame,
            pan_head,
            contact_tol=0.0015,
            elem_a=tilt_trunnion,
            elem_b=pan_ears,
            name="tilt_bearing_contact_folded",
        )
        ctx.expect_origin_gap(
            tilt_frame,
            wall_plate,
            axis="x",
            min_gap=0.10,
            max_gap=0.28,
            name="folded_pose_keeps_frame_off_wall",
        )

    with ctx.pose({head_pan: 1.20}):
        ctx.expect_contact(
            pan_head,
            secondary_arm,
            contact_tol=0.0015,
            elem_a=pan_base,
            elem_b=secondary_pan,
            name="pan_stack_contact_panned",
        )
        ctx.expect_contact(
            tilt_frame,
            pan_head,
            contact_tol=0.0015,
            elem_a=tilt_trunnion,
            elem_b=pan_ears,
            name="tilt_bearing_contact_panned",
        )
        ctx.expect_origin_distance(
            tilt_frame,
            pan_head,
            axes="y",
            min_dist=0.05,
            max_dist=0.06,
            name="pan_pose_swings_tilt_axis_laterally",
        )
        ctx.expect_origin_gap(
            tilt_frame,
            pan_head,
            axis="x",
            min_gap=0.020,
            max_gap=0.024,
            name="pan_pose_reduces_forward_offset_component",
        )

    with ctx.pose({frame_tilt: 0.22}):
        ctx.expect_contact(
            tilt_frame,
            pan_head,
            contact_tol=0.0015,
            elem_a=tilt_trunnion,
            elem_b=pan_ears,
            name="tilt_contact_positive_limit",
        )
        ctx.expect_origin_gap(
            tilt_frame,
            pan_head,
            axis="x",
            min_gap=0.058,
            max_gap=0.062,
            name="tilt_axis_offset_fixed_under_positive_tilt",
        )

    with ctx.pose({frame_tilt: -0.40}):
        ctx.expect_contact(
            tilt_frame,
            pan_head,
            contact_tol=0.0015,
            elem_a=tilt_trunnion,
            elem_b=pan_ears,
            name="tilt_contact_negative_limit",
        )
        ctx.expect_origin_gap(
            tilt_frame,
            pan_head,
            axis="x",
            min_gap=0.058,
            max_gap=0.062,
            name="tilt_axis_offset_fixed_under_negative_tilt",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
