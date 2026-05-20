import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yz_stage")
    
    # Materials
    frame_mat = Material(color=(0.2, 0.2, 0.22), name="frame_metal")
    rail_mat = Material(color=(0.8, 0.8, 0.85), name="rail_steel")
    block_mat = Material(color=(0.1, 0.1, 0.1), name="guide_block")
    platform_mat = Material(color=(0.7, 0.7, 0.7), name="platform_alu")

    # Base
    base = model.part("base")
    
    # Base frame (0.4 x 0.7 x 0.05)
    base_frame_cq = (
        cq.Workplane("XY")
        .box(0.4, 0.7, 0.05)
        .cut(cq.Workplane("XY").box(0.3, 0.6, 0.1))
    )
    base.visual(
        mesh_from_cadquery(base_frame_cq, "base_frame"),
        origin=Origin(xyz=(0, 0, 0.025)),
        material=frame_mat
    )
    
    # Y rails
    y_rail_cq = cq.Workplane("XY").box(0.02, 0.65, 0.02)
    base.visual(
        mesh_from_cadquery(y_rail_cq, "y_rail_left"),
        origin=Origin(xyz=(-0.15, 0.0, 0.06)),
        material=rail_mat
    )
    base.visual(
        mesh_from_cadquery(y_rail_cq, "y_rail_right"),
        origin=Origin(xyz=(0.15, 0.0, 0.06)),
        material=rail_mat
    )
    
    # Y Carriage
    y_carriage = model.part("y_carriage")
    
    # Guide blocks for Y
    y_block_cq = cq.Workplane("XY").box(0.04, 0.06, 0.03)
    for x in [-0.15, 0.15]:
        for y in [-0.15, 0.15]:
            name_x = "m015" if x < 0 else "p015"
            name_y = "m015" if y < 0 else "p015"
            y_carriage.visual(
                mesh_from_cadquery(y_block_cq, f"y_block_{name_x}_{name_y}"),
                origin=Origin(xyz=(x, y, 0.085)),
                material=block_mat
            )
            
    # Base plate for Y carriage
    y_base_plate_cq = cq.Workplane("XY").box(0.34, 0.36, 0.02)
    y_carriage.visual(
        mesh_from_cadquery(y_base_plate_cq, "y_base_plate"),
        origin=Origin(xyz=(0, 0, 0.11)),
        material=frame_mat
    )
            
    # Vertical frame for Y carriage
    y_frame_cq = (
        cq.Workplane("XZ")
        .box(0.3, 0.6, 0.05)
        .cut(cq.Workplane("XZ").box(0.2, 0.5, 0.1))
    )
    y_carriage.visual(
        mesh_from_cadquery(y_frame_cq, "y_frame"),
        origin=Origin(xyz=(0, 0, 0.42)),
        material=frame_mat
    )
    
    # Z rails
    z_rail_cq = cq.Workplane("XY").box(0.02, 0.02, 0.55)
    y_carriage.visual(
        mesh_from_cadquery(z_rail_cq, "z_rail_left"),
        origin=Origin(xyz=(-0.1, 0.035, 0.42)),
        material=rail_mat
    )
    y_carriage.visual(
        mesh_from_cadquery(z_rail_cq, "z_rail_right"),
        origin=Origin(xyz=(0.1, 0.035, 0.42)),
        material=rail_mat
    )
    
    # Y Articulation
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_carriage,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.1, upper=0.1)
    )

    # Z Carriage
    z_carriage = model.part("z_carriage")
    
    # Guide blocks for Z
    z_block_cq = cq.Workplane("XY").box(0.04, 0.03, 0.06)
    for x in [-0.1, 0.1]:
        for z in [-0.15, 0.15]:
            name_x = "m01" if x < 0 else "p01"
            name_z = "m015" if z < 0 else "p015"
            z_carriage.visual(
                mesh_from_cadquery(z_block_cq, f"z_block_{name_x}_{name_z}"),
                origin=Origin(xyz=(x, 0.06, 0.42 + z)),
                material=block_mat
            )
            
    # Backplate for Z carriage
    z_backplate_cq = cq.Workplane("XY").box(0.24, 0.02, 0.36)
    z_carriage.visual(
        mesh_from_cadquery(z_backplate_cq, "z_backplate"),
        origin=Origin(xyz=(0, 0.085, 0.42)),
        material=frame_mat
    )
            
    # Z Platform
    platform_cq = cq.Workplane("XY").box(0.2, 0.2, 0.02)
    z_carriage.visual(
        mesh_from_cadquery(platform_cq, "z_platform"),
        origin=Origin(xyz=(0, 0.195, 0.30)),
        material=platform_mat
    )
    
    # Z Articulation
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.1, upper=0.1)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")
    
    # Check that Y carriage is supported by base rails
    ctx.expect_contact(y_carriage, base, name="y_carriage_rests_on_base_rails")
    
    # Check that Z carriage is supported by Y carriage rails
    ctx.expect_contact(z_carriage, y_carriage, name="z_carriage_rides_on_y_carriage")
    
    # Check poses
    y_joint = object_model.get_articulation("y_slide")
    z_joint = object_model.get_articulation("z_slide")
    
    with ctx.pose({y_joint: 0.1, z_joint: 0.1}):
        ctx.expect_contact(y_carriage, base, name="y_carriage_rests_on_base_rails_extended")
        ctx.expect_contact(z_carriage, y_carriage, name="z_carriage_rides_on_y_carriage_extended")

    return ctx.report()


object_model = build_object_model()