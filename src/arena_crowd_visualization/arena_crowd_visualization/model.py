"""Pure display projection. Never imports or runs the psychology model."""
import math

COLORS = {'REGULAR': (.65,.65,.65), 'CURIOUS': (.15,.55,1.), 'SCARED': (1.,.15,.15),
          'SURPRISED': (1.,.85,.1), 'THREATENING': (.75,.2,1.)}

def number(value):
    value=float(value)
    if not math.isfinite(value): raise ValueError('nonfinite display value')
    return value

def project(document):
    if document.get('schema_version') not in (2,3): raise ValueError('unsupported psychology schema')
    states=[]
    for key,a in document['agents'].items():
        person=document['people'][key]; state=document['states'][key]
        values=a['new']; behavior=a['behavior']
        states.append(dict(id=int(key), name=person['name'], profile=a.get('profile','standard'),
            mode=behavior['mode'], phase=a.get('phase','none'), target_id=int(behavior.get('target_robot_id',0)),
            has_surprise='surprise' in state, curiosity=number(values['curiosity']), fear=number(values['fear']),
            surprise=number(state.get('surprise',0.)),
            **{k.lower():number(a[k]) for k in ('direct_C','direct_F','social_C','social_F')}))
    return sorted(states,key=lambda p:p['id'])

def arrows(document,focus,threshold=.02):
    result=[]; visible=document['agents'].get(str(focus),{}).get('visible_robots',{})
    robots=[]
    for key,v in visible.items():
        c=number(v.get('curiosity_visibility',v['visibility']))*number(v['novelty']); f=number(v['visibility'])*number(v.get('fear_risk',v['risk']))
        robots.append((max(c,f),int(key),c,f,'robot'))
    humans=[]
    for e in document.get('edges',[]):
        if int(e['target_id'])==focus:
            c=number(e['weight'])*number(e['cue_C']);f=number(e['weight'])*number(e['cue_F'])
            humans.append((max(c,f),int(e['source_id']),c,f,'human'))
    for group,limit in ((robots,2),(humans,3)):
        result.extend(sorted((x for x in group if x[0]>=threshold),key=lambda x:(-x[0],x[1]))[:limit])
    return result
